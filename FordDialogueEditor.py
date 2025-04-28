import sys
import json
import math
import os
import logging
import copy
from collections import deque, defaultdict
from functools import partial
from typing import List, Dict, Optional, Tuple, Any, Union, Set
import config
import traceback

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

from PyQt6.QtWidgets import (
    QApplication,
    QGraphicsSceneContextMenuEvent,
    QMainWindow,
    QStyleOptionGraphicsItem,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLineEdit,
    QTextEdit,
    QListWidget,
    QListWidgetItem,
    QFileDialog,
    QLabel,
    QGraphicsView,
    QGraphicsScene,
    QGraphicsItem,
    QGraphicsRectItem,
    QGraphicsTextItem,
    QGraphicsLineItem,
    QMenu,
    QSplitter,
    QGroupBox,
    QFormLayout,
    QMessageBox,
    QInputDialog,
    QComboBox,
    QFrame,
    QGraphicsPathItem,
    QDialog,
    QDialogButtonBox,
    QDockWidget,
    QAbstractItemView,
)
from PyQt6.QtGui import (
    QCloseEvent,
    QPainter,
    QPen,
    QBrush,
    QColor,
    QPolygonF,
    QAction,
    QTransform,
    QWheelEvent,
    QMouseEvent,
    QPalette,
    QPainterPath,
    QKeySequence,
    QUndoStack,
    QUndoCommand,
    QPainterPathStroker,
    QCursor,
    QKeyEvent,
    QTextCursor,
    QFont,
    QTextOption,
)
from PyQt6.QtCore import (
    QPoint,
    Qt,
    QPointF,
    QRectF,
    QLineF,
    QSizeF,
    pyqtSignal,
    QTimer,
    QObject,
    QEvent,
    QItemSelectionModel,
)

def count_crossings_between_nodes(
    u: str,
    v: str,
    nodes_at_level: Dict[int, List[str]],
    node_order_indices: Dict[str, int],
    adj_or_rev_adj: Dict[str, List[str]],
    target_layer_idx: int,
) -> int:
    crossing_count = 0
    target_layer_nodes = nodes_at_level.get(target_layer_idx, [])
    neighbors_u = [
        n
        for n in adj_or_rev_adj.get(u, [])
        if n in node_order_indices and n in target_layer_nodes
    ]
    neighbors_v = [
        n
        for n in adj_or_rev_adj.get(v, [])
        if n in node_order_indices and n in target_layer_nodes
    ]
    if not neighbors_u or not neighbors_v:
        return 0
    indices_u = [node_order_indices[n] for n in neighbors_u]
    indices_v = [node_order_indices[n] for n in neighbors_v]
    for idx_u in indices_u:
        for idx_v in indices_v:
            if idx_u > idx_v:
                crossing_count += 1
    return crossing_count

class BaseUndoCommand(QUndoCommand):
    def __init__(
        self, editor: "DialogueEditor", text: str = "", parent: Optional[QObject] = None
    ):
        super().__init__(text, parent)
        self.editor = editor

class SetCustomPropertyCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        key: str,
        old_value: Any,
        new_value: Any,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        display_name = config.ALLOWED_CUSTOM_PROPERTIES.get(key, {}).get("display", key)
        super().__init__(editor, f"Set Prop '{display_name}' for '{node_id}'", parent)
        self.node_id = node_id
        self.key = key
        self.old_value = old_value
        self.new_value = new_value

    def _set_value(self, value_to_set: Any) -> bool:
        node_data = self.editor.nodes_data.get(self.node_id)
        prop_config = config.ALLOWED_CUSTOM_PROPERTIES.get(self.key)
        if not prop_config:
            logging.warning(
                f"Attempting to set undefined custom property '{self.key}' on node '{self.node_id}'. Ignoring."
            )
            self.setObsolete(True)
            return False

        if node_data and isinstance(node_data.custom_data, dict):
            expected_type_str = prop_config.get("type", "string").lower()
            converted_value = value_to_set
            try:
                if isinstance(value_to_set, str):
                    if expected_type_str == "int":
                        converted_value = int(value_to_set)
                    elif expected_type_str == "float":
                        converted_value = float(value_to_set)
                    elif expected_type_str == "bool":
                        val_lower = value_to_set.lower()
                        if val_lower in ["true", "1", "yes", "on"]:
                            converted_value = True
                        elif val_lower in ["false", "0", "no", "off"]:
                            converted_value = False
                        else:
                            raise ValueError(
                                f"Invalid boolean string: '{value_to_set}'"
                            )
            except (ValueError, TypeError) as e:
                logging.warning(
                    f"Type conversion failed for custom prop '{self.key}' (node '{self.node_id}', value '{value_to_set}', expected '{expected_type_str}'): {e}. Storing as string."
                )
                converted_value = str(value_to_set)

            node_data.custom_data[self.key] = converted_value

            current_selection = self.editor.get_selected_node()
            if current_selection and current_selection.node_data.id == self.node_id:
                widget = self.editor._custom_prop_edits.get(self.key)
                if widget:
                    widget.blockSignals(True)
                    widget.setText(str(converted_value))
                    widget.blockSignals(False)
                    self.editor._original_custom_props[self.key] = str(converted_value)
                else:
                    self.editor.update_properties_panel()

            self.editor._mark_unsaved()
            return True
        else:
            logging.warning(
                f"Node '{self.node_id}' or custom_data not found/valid for SetCustomProp. Obsolete."
            )
            self.setObsolete(True)
            return False

    def redo(self):
        if not self._set_value(self.new_value):
            logging.error(
                f"Redo failed for SetCustomPropertyCommand ({self.node_id}, {self.key})"
            )

    def undo(self):
        if not self._set_value(self.old_value):
            logging.error(
                f"Undo failed for SetCustomPropertyCommand ({self.node_id}, {self.key})"
            )

class DialogueNodeData:
    def __init__(
        self,
        node_id: str = "new_node",
        character: str = "",
        text: str = "",
        pos: Optional[Union[QPointF, List[float], Tuple[float, float]]] = None,
        is_start: bool = False,
        custom_data: Optional[Dict[str, Any]] = None,
    ):
        self.id: str = str(node_id)
        self.character: str = str(character)
        self.text: str = str(text)
        self.choices: List[Tuple[str, str, str]] = []
        self.next_node: Optional[str] = None
        self.is_start_node: bool = bool(is_start)
        self.custom_data: Dict[str, Any] = {}

        for internal_key, prop_config in config.ALLOWED_CUSTOM_PROPERTIES.items():
            default_value = prop_config.get("default", "")
            expected_type_str = prop_config.get("type", "string").lower()
            value_to_use = default_value
            if isinstance(custom_data, dict) and internal_key in custom_data:
                value_to_use = custom_data[internal_key]

            converted_value = value_to_use
            try:
                if value_to_use is not None:
                    if expected_type_str == "int":
                        converted_value = int(value_to_use)
                    elif expected_type_str == "float":
                        converted_value = float(value_to_use)
                    elif expected_type_str == "bool":
                        if isinstance(value_to_use, str):
                            val_lower = value_to_use.lower()
                            if val_lower in ["true", "1", "yes", "on"]:
                                converted_value = True
                            elif val_lower in ["false", "0", "no", "off"]:
                                converted_value = False
                            else:
                                raise ValueError(
                                    f"Invalid boolean string: '{value_to_use}'"
                                )
                        else:
                            converted_value = bool(value_to_use)
                    elif expected_type_str == "string":
                        converted_value = str(value_to_use)
            except (ValueError, TypeError) as e:
                logging.warning(
                    f"Init node '{self.id}', custom prop '{internal_key}': Could not convert initial value '{value_to_use}' to '{expected_type_str}'. Using original/default. Error: {e}"
                )
                converted_value = value_to_use
            self.custom_data[internal_key] = converted_value

        self.pos: QPointF = QPointF(50, 50)
        if isinstance(pos, QPointF):
            self.pos = pos
        elif isinstance(pos, (list, tuple)) and len(pos) == 2:
            try:
                self.pos = QPointF(float(pos[0]), float(pos[1]))
            except (ValueError, TypeError):
                logging.warning(
                    f"Could not parse position data {pos} for node {self.id}. Using default."
                )
        elif pos is not None:
            logging.warning(
                f"Invalid position type {type(pos)} for node {self.id}. Using default."
            )

    def to_dict(self) -> Dict[str, Any]:
        key_map = config.PROJECT_KEY_MAP
        data = {
            key_map.get("id", "id"): self.id,
            key_map.get("character", "character"): self.character,
            key_map.get("text", "text"): self.text,
            key_map.get("pos", "pos"): [self.pos.x(), self.pos.y()],
            key_map.get("is_start_node", "is_start_node"): self.is_start_node,
            key_map.get("custom_data", "custom_data"): self.custom_data.copy(),
        }
        if self.choices:
            data[key_map.get("choices", "choices")] = self.choices
        elif self.next_node:
            data[key_map.get("next_node", "next_node")] = self.next_node
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DialogueNodeData":
        if not isinstance(data, dict):
            raise ValueError("Node data must be a dictionary")
        rev_map = config.PROJECT_REVERSE_KEY_MAP
        node_id = str(data.get(rev_map.get("id", "id"), f"error_id_{id(data)}"))
        character = str(data.get(rev_map.get("character", "character"), ""))
        text = str(data.get(rev_map.get("text", "text"), ""))
        pos_data = data.get(rev_map.get("pos", "pos"))
        is_start = bool(data.get(rev_map.get("is_start_node", "is_start_node"), False))
        loaded_custom_data_dict = data.get(
            rev_map.get("custom_data", "custom_data"), {}
        )
        if not isinstance(loaded_custom_data_dict, dict):
            logging.warning(
                f"Invalid custom_data format for node '{node_id}'. Resetting."
            )
            loaded_custom_data_dict = {}

        node = cls(
            node_id=node_id,
            character=character,
            text=text,
            pos=pos_data,
            is_start=is_start,
            custom_data=loaded_custom_data_dict,
        )

        node.next_node = data.get(rev_map.get("next_node", "next_node"))
        if isinstance(node.next_node, str) and not node.next_node:
            node.next_node = None
        elif node.next_node is None:
            pass
        elif not isinstance(node.next_node, str):
            logging.warning(
                f"Invalid type for next_node ('{node.next_node}') in node '{node_id}'. Setting to None."
            )
            node.next_node = None

        choices_data = data.get(rev_map.get("choices", "choices"), [])
        if isinstance(choices_data, list):
            node.choices = []
            for i, c in enumerate(choices_data):
                if isinstance(c, (list, tuple)) and len(c) == 3:
                    try:
                        node.choices.append((str(c[0]), str(c[1]), str(c[2])))
                    except Exception:
                        logging.warning(
                            f"Could not parse choice {i} elements in node '{node_id}'. Skipping."
                        )
                else:
                    logging.warning(
                        f"Invalid choice format (item {i}: '{c}') for node '{node_id}'. Skipping choice."
                    )
        else:
            logging.warning(
                f"Invalid choices format ('{choices_data}') for node '{node_id}'. Setting to empty list."
            )
            node.choices = []

        if node.choices and node.next_node is not None:
            logging.warning(
                f"Node '{node_id}' has both choices and next_node defined. Clearing next_node."
            )
            node.next_node = None

        return node

class GraphicsNode(QGraphicsItem):
    def __init__(self, node_data: DialogueNodeData, editor: "DialogueEditor"):
        super().__init__()
        self.node_data = node_data
        self.editor = editor
        self.width = config.NODE_WIDTH
        self.height = config.NODE_HEIGHT
        self._is_flashing = False
        self._flash_timer = QTimer()
        self._flash_timer.setSingleShot(True)
        self._flash_timer.setInterval(config.FOCUS_FLASH_DURATION_MS)
        self._flash_timer.timeout.connect(self._stop_flash)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setCacheMode(QGraphicsItem.CacheMode.DeviceCoordinateCache)
        self.setZValue(1)
        self.edges: List[GraphicsEdge] = []
        self._move_start_pos: Optional[QPointF] = None
        self._is_path_highlighted_outgoing = False
        self._is_path_highlighted_incoming = False
        self._is_find_highlighted = False
        self._is_bookmarked = False
        self._base_color_override = None  

    def update_dynamic_visuals(self):
        self._is_bookmarked = self.node_data.id in self.editor.bookmarks
        self._base_color_override = config.NODE_COLORING_RULES.get(
            self.node_data.character
        )
        self.update()

    def set_path_highlight_outgoing(self, highlighted: bool):

        self._is_path_highlighted_outgoing = highlighted
        self.update()

    def set_path_highlight_incoming(self, highlighted: bool):

        self._is_path_highlighted_incoming = highlighted
        self.update()

    def set_find_highlight(self, highlighted: bool):

        self._is_find_highlighted = highlighted
        self.update()

    def toolTip(self) -> str:
        return f"ID: {self.node_data.id}\nChar: {self.node_data.character}\n---\n{self.node_data.text}"

    def boundingRect(self) -> QRectF:
        extra = config.HIGHLIGHT_PEN_WIDTH + config.NODE_BOOKMARK_BORDER_WIDTH
        return QRectF(0, 0, self.width, self.height).adjusted(
            -extra, -extra, extra, extra
        )

    def is_logically_end_node(self) -> bool:
        if not hasattr(self, "node_data"):
            return False
        return not self.node_data.next_node and not self.node_data.choices

    def links_to_start(self) -> Tuple[bool, int, int]:
        if (
            not hasattr(self, "editor")
            or not self.editor.start_node_id
            or not hasattr(self, "node_data")
        ):
            return (False, 0, 0)
        start_id = self.editor.start_node_id
        loop_link_count = 0
        total_links = 0
        if self.node_data.next_node:
            total_links = 1
            if self.node_data.next_node == start_id:
                loop_link_count = 1
        elif self.node_data.choices:
            total_links = len(self.node_data.choices)
            for _, target_id, _ in self.node_data.choices:
                if target_id == start_id:
                    loop_link_count += 1
        return (loop_link_count > 0, loop_link_count, total_links)

    def get_base_color(self) -> QColor:
        if self._base_color_override:
            return QColor(self._base_color_override)

        normal_color = getattr(config, "NODE_NORMAL_COLOR", QColor(60, 60, 90))
        start_color = getattr(config, "NODE_START_COLOR", QColor(0, 100, 0))
        end_color = getattr(config, "NODE_END_COLOR", QColor(100, 0, 0))
        loop_base_color = getattr(config, "NODE_LOOP_COLOR", QColor(100, 100, 0))

        base_color = normal_color
        try:
            if not hasattr(self, "node_data"):
                return base_color
            if self.node_data.is_start_node:
                base_color = start_color
            else:
                is_explicit_loop, loop_count, total_count = self.links_to_start()
                if is_explicit_loop:
                    if total_count > 0:
                        loop_ratio = loop_count / total_count
                        h, s, v, a = loop_base_color.getHsv()
                        new_saturation = max(0, min(255, int(s * loop_ratio)))
                        adjusted_color = QColor.fromHsv(h, new_saturation, v, a)
                        base_color = adjusted_color
                    else:
                        base_color = loop_base_color
                elif (
                    self.is_logically_end_node()
                    and self.editor
                    and self.editor.start_node_id
                ):
                    base_color = loop_base_color
                elif self.is_logically_end_node():
                    base_color = end_color
        except AttributeError as e:
            logging.warning(
                f"AttributeError in get_base_color for node '{getattr(self.node_data, 'id', 'UNKNOWN')}': {e}"
            )
        except Exception as e:
            logging.exception(
                f"Error in get_base_color for node '{getattr(self.node_data, 'id', 'UNKNOWN')}': {e}"
            )

        return QColor(base_color) if not isinstance(base_color, QColor) else base_color

    def paint(
        self,
        painter: QPainter,
        option: QStyleOptionGraphicsItem,
        widget: Optional[QWidget] = None,
    ):
        rect = QRectF(0, 0, self.width, self.height)
        base_color = self.get_base_color()
        current_color = config.FOCUS_FLASH_COLOR if self._is_flashing else base_color
        brush_color = current_color

        if self.isSelected() and not self._is_flashing:
            brightness_factor = getattr(config, "NODE_SELECTED_BRIGHTNESS", 130)
            try:
                brush_color = base_color.lighter(brightness_factor)
            except AttributeError:
                brush_color = QColor(Qt.GlobalColor.magenta)
                logging.error(f"Node {self.node_data.id} base_color was not a QColor.")

        painter.setBrush(QBrush(brush_color))

        default_pen = QPen(
            Qt.GlobalColor.white if self.isSelected() else Qt.GlobalColor.darkGray, 1.5
        )
        painter.setPen(default_pen)

        if self._is_path_highlighted_outgoing or self._is_path_highlighted_incoming:
            highlight_pen = QPen(
                config.NODE_HIGHLIGHT_OUTLINE_COLOR,
                config.HIGHLIGHT_PEN_WIDTH,
                Qt.PenStyle.SolidLine,
            )
            painter.setPen(highlight_pen)
            highlight_fill = QColor(config.NODE_HIGHLIGHT_OUTLINE_COLOR)
            highlight_fill.setAlpha(config.NODE_HIGHLIGHT_FILL_ALPHA)
            painter.setBrush(highlight_fill)
        elif self._is_find_highlighted:
            highlight_pen = QPen(
                config.NODE_FIND_HIGHLIGHT_COLOR,
                config.HIGHLIGHT_PEN_WIDTH,
                Qt.PenStyle.SolidLine,
            )
            painter.setPen(highlight_pen)
            highlight_fill = QColor(config.NODE_FIND_HIGHLIGHT_COLOR)
            highlight_fill.setAlpha(
                config.NODE_HIGHLIGHT_FILL_ALPHA + 20
            )  
            painter.setBrush(highlight_fill)
        else:
            painter.setBrush(
                QBrush(brush_color)
            )  

        painter.drawRoundedRect(rect, 5.0, 5.0)
        painter.setBrush(Qt.BrushStyle.NoBrush)  

        if self._is_bookmarked:
            bookmark_pen = QPen(
                config.NODE_BOOKMARK_BORDER_COLOR,
                config.NODE_BOOKMARK_BORDER_WIDTH,
                Qt.PenStyle.DashLine,
            )
            painter.setPen(bookmark_pen)
            adjust = config.NODE_BOOKMARK_BORDER_WIDTH / 2
            painter.drawRoundedRect(
                rect.adjusted(adjust, adjust, -adjust, -adjust), 4.0, 4.0
            )

        text_margin = 5.0
        id_height = 20.0
        char_height = 15.0
        id_rect = QRectF(
            text_margin, text_margin, self.width - 2 * text_margin, id_height
        )
        char_rect = QRectF(
            text_margin, id_rect.bottom() + 2, self.width - 2 * text_margin, char_height
        )
        text_rect = QRectF(
            text_margin,
            char_rect.bottom() + 5,
            self.width - 2 * text_margin,
            self.height - char_rect.bottom() - 5 - text_margin,
        )

        painter.setPen(QPen(config.NODE_TEXT_COLOR))
        painter.drawText(id_rect, Qt.AlignmentFlag.AlignCenter, self.node_data.id)
        char_preview = f"Char: {self.node_data.character[:15]}{'...' if len(self.node_data.character) > 15 else ''}"
        painter.drawText(
            char_rect,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            char_preview,
        )

        painter.setPen(QPen(config.NODE_TEXT_COLOR))
        cleaned_text = self.node_data.text.replace("\n", " ")
        preview_len = 35
        preview = (
            cleaned_text[:preview_len] + "..."
            if len(cleaned_text) > preview_len
            else cleaned_text
        )
        painter.drawText(
            text_rect,
            Qt.AlignmentFlag.AlignLeft
            | Qt.AlignmentFlag.AlignTop
            | Qt.TextFlag.TextWordWrap,
            preview,
        )

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value: Any) -> Any:
        if (
            change == QGraphicsItem.GraphicsItemChange.ItemPositionChange
            and self.scene()
        ):
            if self._move_start_pos is None:
                self._move_start_pos = self.pos()
        elif change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            new_pos = self.pos()
            if hasattr(self, "node_data"):
                if self.node_data.pos != new_pos:
                    self.node_data.pos = new_pos
            connected_edges = self.editor.get_edges_for_node(self)
            for edge in connected_edges:
                edge.adjust()
        elif change == QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged:
            self._move_start_pos = None
        return super().itemChange(change, value)

    def add_edge(self, edge: "GraphicsEdge"):
        if edge not in self.edges:
            self.edges.append(edge)

    def remove_edge(self, edge: "GraphicsEdge"):
        try:
            self.edges.remove(edge)
        except ValueError:
            pass

    def contextMenuEvent(self, event: QGraphicsSceneContextMenuEvent):
        menu = QMenu()
        if self.editor.start_node_id != self.node_data.id:
            set_start_action = menu.addAction("Set as Start Node")
        else:
            set_start_action = None

        delete_action = menu.addAction("Delete Node")
        menu.addSeparator()
        bookmark_text = "Remove Bookmark" if self._is_bookmarked else "Add Bookmark"
        bookmark_action = menu.addAction(bookmark_text)
        menu.addSeparator()
        highlight_outgoing_action = menu.addAction("Highlight Outgoing Path")
        highlight_incoming_action = menu.addAction("Highlight Incoming Path")
        clear_highlight_action = menu.addAction("Clear Highlights")
        menu.addSeparator()
        copy_action = menu.addAction("Copy Node(s)")  

        action = menu.exec(event.screenPos())

        if action == set_start_action:
            self.editor.set_start_node_cmd(self)
        elif action == delete_action:
            self.editor.delete_node_cmd(self)
        elif action == bookmark_action:
            self.editor.toggle_bookmark_cmd(self.node_data.id)
        elif action == highlight_outgoing_action:
            self.editor.highlight_outgoing_path(self.node_data.id)
        elif action == highlight_incoming_action:
            self.editor.highlight_incoming_path(self.node_data.id)
        elif action == clear_highlight_action:
            self.editor.clear_all_highlights()
        elif action == copy_action:
            self.editor.copy_selected_nodes()

    def flash(self):
        if self._is_flashing:
            self._flash_timer.start()
            return
        self._is_flashing = True
        self._flash_timer.start()
        self.update()

    def _stop_flash(self):
        self._is_flashing = False
        self.update()

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            pass
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        super().mouseReleaseEvent(event)
        if (
            event.button() == Qt.MouseButton.LeftButton
            and self._move_start_pos is not None
        ):
            current_pos = self.pos()
            start_pos = self._move_start_pos
            self._move_start_pos = None
            if (
                QLineF(start_pos, current_pos).length()
                > QApplication.startDragDistance()
            ):
                logging.debug(
                    f"Node '{self.node_data.id}' moved from {start_pos} to {current_pos}. Creating command."
                )
                cmd = MoveNodeCommand(self, start_pos, current_pos, self.editor)
                self.editor.undo_stack.push(cmd)

    def mouseDoubleClickEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            logging.debug(f"Node '{self.node_data.id}' double-clicked.")
            if self.editor:
                self.editor.request_text_edit_focus(self.node_data.id)
            event.accept()
        else:
            super().mouseDoubleClickEvent(event)

class GraphicsEdge(QGraphicsItem):
    def __init__(
        self,
        source_node: GraphicsNode,
        dest_node: GraphicsNode,
        editor: "DialogueEditor",
        choice_text: Optional[str] = None,
    ):
        super().__init__()
        self.source = source_node
        self.dest = dest_node
        self.choice_text = choice_text
        self.editor = editor
        self._source_pos = QPointF()
        self._dest_pos = QPointF()
        self._control1 = QPointF()
        self._control2 = QPointF()
        self._path = QPainterPath()
        self.setZValue(0)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)

        pen_width = config.EDGE_PEN_WIDTH
        self.pen = QPen(
            config.EDGE_DEFAULT_COLOR,
            pen_width,
            Qt.PenStyle.SolidLine,
            Qt.PenCapStyle.RoundCap,
            Qt.PenJoinStyle.RoundJoin,
        )
        self.selected_pen = QPen(
            config.EDGE_SELECTED_COLOR,
            pen_width + 0.5,
            Qt.PenStyle.SolidLine,
            Qt.PenCapStyle.RoundCap,
            Qt.PenJoinStyle.RoundJoin,
        )
        self.highlight_pen = QPen(
            config.EDGE_HIGHLIGHT_COLOR,
            pen_width + 0.5,
            Qt.PenStyle.SolidLine,
            Qt.PenCapStyle.RoundCap,
            Qt.PenJoinStyle.RoundJoin,
        )
        self.find_highlight_pen = QPen(
            config.EDGE_FIND_HIGHLIGHT_COLOR,
            pen_width + 0.5,
            Qt.PenStyle.SolidLine,
            Qt.PenCapStyle.RoundCap,
            Qt.PenJoinStyle.RoundJoin,
        )

        self.arrow_size = config.ARROW_SIZE
        self._is_path_highlighted = False
        self._is_find_highlighted = False

        if source_node:
            source_node.add_edge(self)
        self.adjust()

    def set_path_highlight(self, highlighted: bool):

        self._is_path_highlighted = highlighted
        self.update()

    def set_find_highlight(self, highlighted: bool):

        self._is_find_highlighted = highlighted
        self.update()

    def shape(self) -> QPainterPath:
        if self._path.isEmpty():
            return QPainterPath()
        stroker = QPainterPathStroker()
        stroker.setWidth(max(10.0, self.pen.widthF() + 6.0))
        return stroker.createStroke(self._path)

    def boundingRect(self) -> QRectF:
        if self._path.isEmpty():
            return QRectF()
        pen_width = self.pen.widthF()
        extra = (pen_width / 2.0) + self.arrow_size + 5.0
        return self._path.boundingRect().adjusted(-extra, -extra, extra, extra)

    def calculate_path(self) -> QPainterPath:
        if not self.source or not self.dest:
            return QPainterPath()
        offset = 0.0
        if self.source == self.dest:
            logging.warning(
                "Attempting to draw self-loop edge visually. This might look odd."
            )
            offset = 20.0

        p1 = self.source.scenePos() + QPointF(
            self.source.width / 2.0, self.source.height + offset
        )
        p2 = self.dest.scenePos() + QPointF(self.dest.width / 2.0, -offset)

        path = QPainterPath()
        path.moveTo(p1)

        dy = p2.y() - p1.y()
        dx = p2.x() - p1.x()
        control_offset_y = abs(dy) * 0.4
        control_offset_x = dx * 0.1

        c1 = p1 + QPointF(control_offset_x, control_offset_y)
        c2 = p2 + QPointF(-control_offset_x, -control_offset_y)

        self._source_pos = p1
        self._dest_pos = p2
        self._control1 = c1
        self._control2 = c2

        path.cubicTo(self._control1, self._control2, self._dest_pos)
        return path

    def adjust(self):
        if not self.source or not self.dest:
            self.prepareGeometryChange()
            self._path = QPainterPath()
            self.update()
            return
        self.prepareGeometryChange()
        self._path = self.calculate_path()
        self.update()

    def paint(
        self,
        painter: QPainter,
        option: QStyleOptionGraphicsItem,
        widget: Optional[QWidget] = None,
    ):
        if self._path.isEmpty() or not self.source or not self.dest:
            return

        if self._is_path_highlighted:
            current_pen = self.highlight_pen
        elif self._is_find_highlighted:
            current_pen = self.find_highlight_pen
        elif self.isSelected():
            current_pen = self.selected_pen
        else:
            current_pen = self.pen

        painter.setPen(current_pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawPath(self._path)

        try:
            path_len = self._path.length()
            arrow_margin = 0.0
            arrow_size = self.arrow_size
            if path_len < arrow_margin + arrow_size:
                return

            tip_length = path_len - arrow_margin
            arrow_tip_percent = self._path.percentAtLength(tip_length)
            arrow_tip = self._path.pointAtPercent(arrow_tip_percent)
            angle_check_percent = self._path.percentAtLength(max(0.0, tip_length - 1.0))
            if angle_check_percent >= 1.0:
                angle_check_percent = arrow_tip_percent

            angle_deg = self._path.angleAtPercent(angle_check_percent)
            angle_rad = math.radians(-angle_deg)
            angle_offset = math.pi / 6

            dx1 = math.cos(angle_rad + angle_offset) * arrow_size
            dy1 = math.sin(angle_rad + angle_offset) * arrow_size
            arrow_p1 = arrow_tip - QPointF(dx1, dy1)
            dx2 = math.cos(angle_rad - angle_offset) * arrow_size
            dy2 = math.sin(angle_rad - angle_offset) * arrow_size
            arrow_p2 = arrow_tip - QPointF(dx2, dy2)

            arrowhead = QPolygonF([arrow_tip, arrow_p1, arrow_p2])

            if not arrowhead.boundingRect().isEmpty():
                painter.setBrush(QBrush(current_pen.color()))
                painter.setPen(QPen(current_pen.color(), 1))
                painter.drawPolygon(arrowhead)
            else:
                logging.warning(
                    f"Skipping drawing invalid arrowhead polygon (offset) for {self.source.node_data.id}->{self.dest.node_data.id}"
                )

        except Exception as e:
            logging.exception(
                f"Error drawing edge arrow for {getattr(self.source.node_data, 'id', 'N/A')}->{getattr(self.dest.node_data, 'id', 'N/A')}: {e}"
            )

    def remove(self):
        if self.source:
            self.source.remove_edge(self)
        if self.scene():
            try:
                self.scene().removeItem(self)
            except Exception as e:
                logging.exception(f"Error removing edge item from scene: {e}")

    def contextMenuEvent(self, event: QGraphicsSceneContextMenuEvent):
        menu = QMenu()
        delete_action = menu.addAction("Delete Connection")
        action = menu.exec(event.screenPos())

        if action == delete_action:
            if self.editor:
                self.editor.delete_edge_cmd(self)
            else:
                logging.error(
                    "Edge context menu couldn't find editor reference to delete."
                )

class ZoomPanGraphicsView(QGraphicsView):
    edgeDragStarted = pyqtSignal(GraphicsNode, QPointF)
    edgeDragMoved = pyqtSignal(QPointF)
    edgeDragEnded = pyqtSignal(QPointF)
    edgeDragCancelled = pyqtSignal()

    def __init__(
        self,
        scene: QGraphicsScene,
        editor: "DialogueEditor",
        parent: Optional[QWidget] = None,
    ):
        super().__init__(scene, parent)
        self.editor = editor
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._zoom_factor_base = config.VIEW_ZOOM_FACTOR
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def wheelEvent(self, event: QWheelEvent):
        angle = event.angleDelta().y()
        if angle == 0:
            event.ignore()
            return
        factor = self._zoom_factor_base if angle > 0 else 1.0 / self._zoom_factor_base
        current_scale = self.transform().m11()
        min_scale = getattr(config, "VIEW_MIN_ZOOM", 0.1)
        max_scale = getattr(config, "VIEW_MAX_ZOOM", 10.0)
        if (factor > 1.0 and current_scale * factor > max_scale) or (
            factor < 1.0 and current_scale * factor < min_scale
        ):
            event.ignore()
            return
        self.scale(factor, factor)
        event.accept()

    def keyPressEvent(self, event: QKeyEvent):
        if not self.editor:
            super().keyPressEvent(event)
            return

        accepted = False
        if event.key() == Qt.Key.Key_Delete or event.key() == Qt.Key.Key_Backspace:
            self.editor.delete_selected_items_cmd()
            accepted = True
        elif event.matches(QKeySequence.StandardKey.New):
            scene_pos = self.mapToScene(self.viewport().rect().center())
            self.editor.add_node_cmd(
                pos=scene_pos,
                character=self.editor.get_default_character_for_new_node(),
            )
            accepted = True
        elif event.matches(QKeySequence.StandardKey.Copy):
            self.editor.copy_selected_nodes()
            accepted = True
        elif event.matches(QKeySequence.StandardKey.Paste):
            self.editor.paste_nodes()
            accepted = True
        elif (
            event.modifiers() == Qt.KeyboardModifier.ControlModifier
            and event.key() == Qt.Key.Key_L
        ):
            self.editor.link_selected_nodes_cmd()
            accepted = True
        elif event.key() == Qt.Key.Key_L and not self.editor.is_drawing_edge:
            selected_nodes = self.editor.get_selected_nodes()
            if len(selected_nodes) == 1:
                source_node = selected_nodes[0]
                line_start_pos = source_node.scenePos() + QPointF(
                    source_node.width / 2, source_node.height
                )
                self.edgeDragStarted.emit(source_node, line_start_pos)
                accepted = True
            else:
                logging.info(
                    "Press L with exactly one node selected to start drawing an edge."
                )
        elif event.key() == Qt.Key.Key_Escape:
            if self.editor.is_drawing_edge:
                self.edgeDragCancelled.emit()
                accepted = True
            else:  
                self.editor.clear_all_highlights()
                if self.scene():
                    self.scene().clearSelection()
                accepted = True

        if accepted:
            event.accept()
        else:
            super().keyPressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.editor and self.editor.is_drawing_edge:
            self.edgeDragMoved.emit(self.mapToScene(event.pos()))
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if self.editor and self.editor.is_drawing_edge:
            if event.button() == Qt.MouseButton.LeftButton:
                self.edgeDragEnded.emit(self.mapToScene(event.pos()))
            event.accept()
        else:
            super().mouseReleaseEvent(event)

    def contextMenuEvent(self, event: QMouseEvent) -> None:
        if self.editor and self.editor.is_drawing_edge:
            event.accept()
            return

        item = self.itemAt(event.pos())
        menu = QMenu(self)
        scene_pos = self.mapToScene(event.pos())

        if item is None:
            add_node_action = menu.addAction("Add Node Here")
            menu.addSeparator()
            paste_action = menu.addAction("Paste Node(s)")
            paste_action.setEnabled(self.editor.has_clipboard_data())
            action = menu.exec(event.globalPos())
            if action == add_node_action:
                if self.editor:
                    self.editor.add_node_cmd(
                        pos=scene_pos,
                        character=self.editor.get_default_character_for_new_node(),
                    )
            elif action == paste_action:
                if self.editor:
                    self.editor.paste_nodes(scene_pos)

        else:  
            super().contextMenuEvent(event)

class ClickAwareListWidget(QListWidget):
    focusRequested = pyqtSignal(str)
    editRequested = pyqtSignal(QListWidgetItem)
    nextToChoiceRequested = pyqtSignal(QListWidgetItem)

    def __init__(self, editor: "DialogueEditor", parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.editor = editor
        self.focus_click_threshold_percent = getattr(
            config, "LIST_FOCUS_CLICK_THRESHOLD_PERCENT", 60
        )

    def mousePressEvent(self, event: QMouseEvent):
        item = self.itemAt(event.pos())
        if not item or event.button() != Qt.MouseButton.LeftButton:
            super().mousePressEvent(event)
            return

        item_data = item.data(Qt.ItemDataRole.UserRole)
        if (
            not item_data
            or not isinstance(item_data, (list, tuple))
            or len(item_data) < 1
        ):
            super().mousePressEvent(event)
            return

        item_rect = self.visualItemRect(item)
        click_x = event.pos().x()
        focus_threshold_x = item_rect.left() + item_rect.width() * (
            self.focus_click_threshold_percent / 100.0
        )

        connection_type = item_data[0]
        target_id = None
        try:
            if connection_type == config.CONN_TYPE_CHOICE:
                target_id = item_data[2]
            elif connection_type == config.CONN_TYPE_NEXT:
                target_id = item_data[1]
            elif connection_type == config.CONN_TYPE_IMPLICIT_LOOP:
                target_id = item_data[1]
        except IndexError:
            logging.warning(
                f"IndexError accessing item data for focus click: {item_data}"
            )
            super().mousePressEvent(event)
            return

        if click_x > focus_threshold_x and target_id is not None:
            logging.debug(f"Focus click detected! Emitting focusRequested({target_id})")
            self.focusRequested.emit(target_id)
            self.setCurrentItem(None)  
            event.accept()
        else:
            logging.debug("Non-focus click detected. Passing to base class.")
            super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent):
        item = self.itemAt(event.pos())
        if item and event.button() == Qt.MouseButton.LeftButton:
            item_data = item.data(Qt.ItemDataRole.UserRole)
            if item_data and isinstance(item_data, (list, tuple)):
                connection_type = item_data[0]
                if connection_type == config.CONN_TYPE_CHOICE:
                    self.editRequested.emit(item)
                    event.accept()
                    return
                elif connection_type == config.CONN_TYPE_NEXT:
                    self.nextToChoiceRequested.emit(item)
                    event.accept()
                    return
        super().mouseDoubleClickEvent(event)

class ValidationResultsDialog(QDialog):
    focusRequested = pyqtSignal(str)  

    def __init__(
        self,
        results: List[Tuple[str, str, Optional[str]]],
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Validation Results")
        self.setMinimumWidth(500)
        self.setMinimumHeight(300)

        layout = QVBoxLayout(self)
        self.results_list = QListWidget()
        self.results_list.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )
        layout.addWidget(self.results_list)

        for level, message, node_id in results:
            item = QListWidgetItem(f"[{level}] {message}")
            if level == "ERROR":
                item.setForeground(QColor("red"))
            elif level == "WARNING":
                item.setForeground(QColor("orange"))
            if node_id:
                item.setData(
                    Qt.ItemDataRole.UserRole, node_id
                )  
            self.results_list.addItem(item)

        self.results_list.itemDoubleClicked.connect(self._on_item_double_clicked)

        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        button_box.accepted.connect(self.accept)
        layout.addWidget(button_box)

    def _on_item_double_clicked(self, item: QListWidgetItem):
        node_id = item.data(Qt.ItemDataRole.UserRole)
        if node_id:
            self.focusRequested.emit(node_id)

class AddNodeCommand(BaseUndoCommand):
    def __init__(
        self,
        node_data: DialogueNodeData,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Add Node '{node_data.id}'", parent)
        self.node_data = node_data
        self.graphics_node: Optional[GraphicsNode] = None
        self.was_first_node = not bool(editor.nodes_data)

    def redo(self):
        if self.node_data.id in self.editor.nodes_data:
            logging.warning(
                f"Node ID '{self.node_data.id}' already exists. AddNodeCommand obsolete."
            )
            self.setObsolete(True)
            return

        self.graphics_node = self.editor._add_node_internal(self.node_data)
        if not self.graphics_node:
            logging.error(
                f"Failed to add node '{self.node_data.id}' internally. Command obsolete."
            )
            self.setObsolete(True)
            return

        set_as_start = self.node_data.is_start_node or self.was_first_node
        if set_as_start:
            if not self.editor._set_start_node_internal(self.graphics_node, force=True):
                logging.warning(
                    f"Failed to set new node '{self.node_data.id}' as start during redo."
                )
            actual_id_after_set = (
                self.graphics_node.node_data.id
            )  
            self.node_data.is_start_node = (
                self.editor.start_node_id == actual_id_after_set
            )

        self.editor._mark_unsaved()
        self.editor.update_properties_panel()
        self.editor.scene.clearSelection()
        if self.graphics_node:
            self.graphics_node.setSelected(True)
        self.editor.update_dynamic_node_visuals(
            self.node_data.id
        )  

    def undo(self):
        node_id_to_delete = (
            self.graphics_node.node_data.id if self.graphics_node else self.node_data.id
        )
        if self.editor._delete_node_internal(node_id_to_delete):
            self.editor._mark_unsaved()
            self.editor.update_properties_panel()
        else:
            logging.error(
                f"Failed to delete node '{node_id_to_delete}' during undo. Command might be obsolete."
            )
            self.setObsolete(True)

class DeleteNodeCommand(BaseUndoCommand):
    def __init__(
        self, node_id: str, editor: "DialogueEditor", parent: Optional[QObject] = None
    ):
        super().__init__(editor, f"Delete Node '{node_id}'", parent)
        self.node_id = node_id
        self.node_data_dict: Optional[Dict[str, Any]] = None
        self.was_start_node: bool = False
        self.incoming_connections: List[
            Tuple[str, str, Optional[int], Optional[Tuple]]
        ] = []
        self.was_bookmarked: bool = False

    def redo(self):
        gnode_to_delete = self.editor.graphics_nodes.get(self.node_id)
        if not gnode_to_delete or self.node_id not in self.editor.nodes_data:
            logging.warning(
                f"Node '{self.node_id}' not found for deletion redo. Command obsolete."
            )
            self.setObsolete(True)
            return

        self.node_data_dict = gnode_to_delete.node_data.to_dict()
        self.was_start_node = self.editor.start_node_id == self.node_id
        self.was_bookmarked = self.node_id in self.editor.bookmarks

        self.incoming_connections = []
        for src_id, src_data in self.editor.nodes_data.items():
            if src_id == self.node_id:
                continue
            if src_data.next_node == self.node_id:
                self.incoming_connections.append((src_id, "next", None, None))
            else:
                for i, choice in enumerate(src_data.choices):
                    if choice[1] == self.node_id:
                        self.incoming_connections.append((src_id, "choice", i, choice))

        if self.was_bookmarked:
            self.editor.bookmarks.discard(self.node_id)
            self.editor.update_bookmark_menu()

        if not self.editor._delete_node_internal(self.node_id):
            logging.error(
                f"Internal deletion of node '{self.node_id}' failed during redo. Command obsolete."
            )
            self.setObsolete(True)

            if self.was_bookmarked:
                self.editor.bookmarks.add(self.node_id)
            return

        self.editor._mark_unsaved()
        self.editor.update_properties_panel()

    def undo(self):
        if not self.node_data_dict:
            logging.warning("No node data stored for undo deletion. Command obsolete.")
            self.setObsolete(True)
            return
        if self.node_id in self.editor.nodes_data:
            logging.warning(
                f"Node ID '{self.node_id}' already exists during undo deletion. Command obsolete."
            )
            self.setObsolete(True)
            return

        try:
            restored_data = DialogueNodeData.from_dict(self.node_data_dict)
        except Exception as e:
            logging.exception(
                f"Error deserializing node data during undo deletion for '{self.node_id}': {e}. Obsolete."
            )
            self.setObsolete(True)
            return

        restored_gnode = self.editor._add_node_internal(restored_data)
        if not restored_gnode:
            logging.error(
                f"Failed to internally re-add node '{self.node_id}' during undo. Obsolete."
            )
            self.setObsolete(True)
            return

        for src_id, conn_type, index_or_none, choice_data in self.incoming_connections:
            if src_id in self.editor.nodes_data:
                src_node_data = self.editor.nodes_data[src_id]
                try:
                    if conn_type == "next":
                        src_node_data.next_node = self.node_id
                        src_node_data.choices = []
                    elif conn_type == "choice" and choice_data:
                        src_node_data.next_node = None
                        if not isinstance(src_node_data.choices, list):
                            src_node_data.choices = []
                        if isinstance(index_or_none, int) and 0 <= index_or_none <= len(
                            src_node_data.choices
                        ):
                            src_node_data.choices.insert(index_or_none, choice_data)
                        else:
                            logging.warning(
                                f"Invalid index ({index_or_none}) restoring choice to {src_id}. Appending."
                            )
                            src_node_data.choices.append(choice_data)
                except Exception as e:
                    logging.exception(
                        f"Error restoring incoming connection to {src_id}: {e}"
                    )

        if self.was_start_node:
            if not self.editor._set_start_node_internal(restored_gnode, force=True):
                logging.warning(
                    f"Failed to restore start node status for '{self.node_id}' during undo."
                )
            if self.editor.start_node_id == restored_data.id:
                restored_data.is_start_node = True
            else:
                restored_data.is_start_node = False

        if self.was_bookmarked:
            self.editor.bookmarks.add(self.node_id)
            self.editor.update_bookmark_menu()
            restored_gnode.update_dynamic_visuals()

        self.editor.redraw_all_edges()
        self.editor._mark_unsaved()
        self.editor.update_properties_panel()
        self.editor.update_dynamic_node_visuals(self.node_id)

class MoveNodeCommand(BaseUndoCommand):
    def __init__(
        self,
        graphics_node: GraphicsNode,
        old_pos: QPointF,
        new_pos: QPointF,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        self.node_id = graphics_node.node_data.id
        super().__init__(editor, f"Move Node '{self.node_id}'", parent)
        self.old_pos = QPointF(old_pos)
        self.new_pos = QPointF(new_pos)

    def _set_pos_and_update(self, pos_to_set: QPointF) -> bool:
        gnode = self.editor.graphics_nodes.get(self.node_id)
        if gnode:
            gnode.setPos(pos_to_set)
            return True
        else:
            logging.warning(
                f"Node '{self.node_id}' not found during move command. Obsolete."
            )
            self.setObsolete(True)
            return False

    def redo(self):
        logging.debug(f"Redo MoveNode: {self.node_id} to {self.new_pos}")
        if self._set_pos_and_update(self.new_pos):
            self.editor._mark_unsaved()

    def undo(self):
        logging.debug(f"Undo MoveNode: {self.node_id} to {self.old_pos}")
        if self._set_pos_and_update(self.old_pos):
            self.editor._mark_unsaved()

class SetNodeTextCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        old_text: str,
        new_text: str,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Edit Node Text '{node_id}'", parent)
        self.node_id = node_id
        self.old_text = old_text
        self.new_text = new_text

    def _set_text(self, text_to_set: str) -> bool:
        node_data = self.editor.nodes_data.get(self.node_id)
        if node_data:
            node_data.text = text_to_set
            if self.node_id in self.editor.graphics_nodes:
                self.editor.graphics_nodes[self.node_id].update()

            selected_node = self.editor.get_selected_node()
            if selected_node and selected_node.node_data.id == self.node_id:
                self.editor.node_text_edit.blockSignals(True)
                if self.editor.node_text_edit.toPlainText() != text_to_set:
                    self.editor.node_text_edit.setPlainText(text_to_set)
                self.editor.node_text_edit.blockSignals(False)
                self.editor._original_node_text = text_to_set

            self.editor._mark_unsaved()
            return True
        else:
            logging.warning(
                f"Node '{self.node_id}' not found for text change. Command obsolete."
            )
            self.setObsolete(True)
            return False

    def redo(self):
        self._set_text(self.new_text)

    def undo(self):
        self._set_text(self.old_text)

    def id(self) -> int:
        base_id = 1000
        try:
            hash_val = hash(self.node_id) % 2147483647
        except TypeError:
            hash_val = 0
        return base_id + hash_val

    def mergeWith(self, other: QUndoCommand) -> bool:
        if not isinstance(other, SetNodeTextCommand) or other.id() != self.id():
            return False
        self.new_text = other.new_text
        logging.debug(f"Merged text command for node {self.node_id}")
        return True

class ApplyLayoutCommand(BaseUndoCommand):
    def __init__(
        self,
        old_positions_dict: Dict[str, QPointF],
        new_positions_dict: Dict[str, QPointF],
        editor: "DialogueEditor",
        text: str = "Apply Auto Layout",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, text, parent)
        self.old_positions = old_positions_dict.copy()
        self.new_positions = new_positions_dict.copy()
        self.applied_state: Optional[str] = None  

    def _apply_positions(self, positions_to_apply: Dict[str, QPointF]):
        if not hasattr(self.editor, "graphics_nodes") or not hasattr(
            self.editor, "redraw_all_edges"
        ):
            logging.error("Editor references missing in ApplyLayoutCommand.")
            self.setObsolete(True)
            return

        nodes_updated = 0
        for node_id, target_pos in positions_to_apply.items():
            gnode = self.editor.graphics_nodes.get(node_id)
            if gnode:
                current_pos = gnode.pos()
                if (
                    abs(current_pos.x() - target_pos.x()) > 0.01
                    or abs(current_pos.y() - target_pos.y()) > 0.01
                ):
                    gnode.setPos(target_pos)
                    nodes_updated += 1
            else:
                logging.warning(
                    f"Node '{node_id}' not found during layout command execution."
                )
        logging.info(f"Layout Command: Applied positions for {nodes_updated} nodes.")
        self.editor.redraw_all_edges()
        if hasattr(self.editor, "_fit_view_after_layout"):
            self.editor._fit_view_after_layout()

    def redo(self):
        if self.applied_state != "new":
            logging.info("Redo ApplyLayoutCommand...")
            self._apply_positions(self.new_positions)
            self.editor._mark_unsaved()
            self.applied_state = "new"
        else:
            logging.debug("Redo ApplyLayoutCommand: Already in target state.")

    def undo(self):
        if self.applied_state != "old":
            logging.info("Undo ApplyLayoutCommand...")
            self._apply_positions(self.old_positions)
            self.editor._mark_unsaved()
            self.applied_state = "old"
        else:
            logging.debug("Undo ApplyLayoutCommand: Already in target state.")

class SetNodeCharCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        old_char: str,
        new_char: str,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Edit Node Character '{node_id}'", parent)
        self.node_id = node_id
        self.old_char = old_char
        self.new_char = new_char

    def _set_char(self, char_to_set: str) -> bool:
        node_data = self.editor.nodes_data.get(self.node_id)
        if node_data:
            node_data.character = char_to_set
            gnode = self.editor.graphics_nodes.get(self.node_id)
            if gnode:
                gnode.update()
                self.editor.update_dynamic_node_visuals(
                    self.node_id
                )  

            selected_node = self.editor.get_selected_node()
            if selected_node and selected_node.node_data.id == self.node_id:
                self.editor.node_char_edit.blockSignals(True)
                self.editor.node_char_edit.setText(char_to_set)
                self.editor.node_char_edit.blockSignals(False)
                self.editor._original_node_char = char_to_set

            self.editor._mark_unsaved()
            return True
        else:
            logging.warning(
                f"Node '{self.node_id}' not found for char change. Command obsolete."
            )
            self.setObsolete(True)
            return False

    def redo(self):
        self._set_char(self.new_char)

    def undo(self):
        self._set_char(self.old_char)

    def id(self) -> int:
        base_id = 2000
        try:
            hash_val = hash(self.node_id) % 2147483647
        except TypeError:
            hash_val = 0
        return base_id + hash_val

    def mergeWith(self, other: QUndoCommand) -> bool:
        if not isinstance(other, SetNodeCharCommand) or other.id() != self.id():
            return False
        self.new_char = other.new_char
        logging.debug(f"Merged character command for node {self.node_id}")
        return True

class SetStartNodeCommand(BaseUndoCommand):
    def __init__(
        self,
        new_start_id: str,
        old_start_id: Optional[str],
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Set Start Node to '{new_start_id}'", parent)
        self.new_start_node_id = new_start_id
        self.old_start_node_id = old_start_id
        self.new_node_was_renamed_from: Optional[str] = None
        self.old_node_needs_rename_from_start: bool = (
            old_start_id == config.START_NODE_EXPORT_ID
        )

    def redo(self):
        gnode_to_set = self.editor.graphics_nodes.get(self.new_start_node_id)
        if not gnode_to_set:
            logging.warning(
                f"Target start node '{self.new_start_node_id}' not found for redo. Obsolete."
            )
            self.setObsolete(True)
            return

        original_id_before_set = gnode_to_set.node_data.id
        if self.editor._set_start_node_internal(gnode_to_set, force=True):
            final_id_after_set = gnode_to_set.node_data.id
            if (
                final_id_after_set == config.START_NODE_EXPORT_ID
                and original_id_before_set != config.START_NODE_EXPORT_ID
            ):
                self.new_node_was_renamed_from = original_id_before_set
                logging.info(
                    f"SetStartNode redo: Renamed '{original_id_before_set}' to '{final_id_after_set}'."
                )
            else:
                self.new_node_was_renamed_from = None

            self.editor._mark_unsaved()
            self.editor.update_properties_panel()
        else:
            logging.error(
                f"Internal setting of start node '{self.new_start_node_id}' failed during redo. Obsolete."
            )
            self.setObsolete(True)

    def undo(self):

        id_that_was_set_by_redo = (
            config.START_NODE_EXPORT_ID
            if self.new_node_was_renamed_from
            else self.new_start_node_id
        )

        current_actual_start_id = self.editor.start_node_id

        if (
            self.new_node_was_renamed_from
            and current_actual_start_id == config.START_NODE_EXPORT_ID
        ):
            start_gnode = self.editor.graphics_nodes.get(config.START_NODE_EXPORT_ID)
            if start_gnode:
                logging.info(
                    f"SetStartNode undo: Attempting rename '{config.START_NODE_EXPORT_ID}' back to '{self.new_node_was_renamed_from}'."
                )
                if not self.editor._perform_node_rename_internal(
                    start_gnode,
                    config.START_NODE_EXPORT_ID,
                    self.new_node_was_renamed_from,
                ):
                    logging.error(
                        f"Failed to rename '{config.START_NODE_EXPORT_ID}' back to '{self.new_node_was_renamed_from}' during undo."
                    )
                else:

                    id_that_was_set_by_redo = self.new_node_was_renamed_from
            else:
                logging.error(
                    f"Inconsistency during start node undo - node '{config.START_NODE_EXPORT_ID}' not found for rename."
                )

        if id_that_was_set_by_redo in self.editor.graphics_nodes:
            self.editor._unset_start_node_visuals(id_that_was_set_by_redo)

        self.editor.start_node_id = None  

        if self.old_start_node_id:
            gnode_to_restore = self.editor.graphics_nodes.get(self.old_start_node_id)
            if gnode_to_restore:
                logging.info(
                    f"SetStartNode undo: Restoring start node to '{self.old_start_node_id}'."
                )

                if not self.editor._set_start_node_internal(
                    gnode_to_restore, force=True
                ):
                    logging.warning(
                        f"Failed to restore original start node '{self.old_start_node_id}' during undo."
                    )
            else:
                logging.warning(
                    f"Original start node '{self.old_start_node_id}' graphics not found during undo."
                )
        else:
            logging.info("SetStartNode undo: No previous start node to restore.")

        self.editor.update_properties_panel()
        self.editor.redraw_all_edges()  
        self.editor._mark_unsaved()

class AddChoiceCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        text: str,
        target_id: str,
        preset: str,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        desc = (
            f"'Next' to '{target_id}'"
            if text == config.DEFAULT_CHOICE_TEXT
            else f"Choice '{text}'"
        )
        super().__init__(editor, f"Add {desc} to '{node_id}'", parent)
        self.node_id = node_id
        self.choice_tuple = (str(text), str(target_id), str(preset))
        self.was_next: Optional[str] = None
        self.overwritten_choices: List[Tuple[str, str, str]] = []

    def redo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if not node_data:
            logging.warning(
                f"Source node '{self.node_id}' not found for AddChoice redo. Obsolete."
            )
            self.setObsolete(True)
            return

        self.was_next = node_data.next_node
        self.overwritten_choices = (
            list(node_data.choices) if isinstance(node_data.choices, list) else []
        )

        is_default_choice = self.choice_tuple[0] == config.DEFAULT_CHOICE_TEXT
        if is_default_choice:
            node_data.next_node = self.choice_tuple[1]
            if node_data.choices:
                logging.debug(
                    f"AddChoice (as Next) redo: Clearing {len(node_data.choices)} existing choices."
                )
            node_data.choices = []
        else:
            if node_data.next_node is not None:
                logging.debug(
                    f"AddChoice redo: Clearing existing next_node ('{node_data.next_node}')."
                )
            node_data.next_node = None
            if not isinstance(node_data.choices, list):
                node_data.choices = []
            if self.choice_tuple not in node_data.choices:
                node_data.choices.append(self.choice_tuple)
            else:
                logging.warning(
                    f"Choice {self.choice_tuple} already exists on node {self.node_id}. AddChoice redo."
                )

        gnode = self.editor.graphics_nodes.get(self.node_id)
        if gnode:
            self.editor.draw_edges_for_node(gnode)
            gnode.update()
        self.editor.update_properties_panel()
        if self.editor.choices_list:
            self.editor.choices_list.update()
        self.editor._mark_unsaved()
        self.editor.update_dynamic_node_visuals(self.node_id)

    def undo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if not node_data:
            logging.warning(
                f"Source node '{self.node_id}' not found for AddChoice undo. Obsolete."
            )
            self.setObsolete(True)
            return

        node_data.next_node = self.was_next
        node_data.choices = (
            list(self.overwritten_choices)
            if isinstance(self.overwritten_choices, list)
            else []
        )
        logging.debug(
            f"AddChoice undo: Restored next='{node_data.next_node}', choices={node_data.choices}"
        )

        gnode = self.editor.graphics_nodes.get(self.node_id)
        if gnode:
            self.editor.draw_edges_for_node(gnode)
            gnode.update()
        self.editor.update_properties_panel()
        self.editor._mark_unsaved()
        self.editor.update_dynamic_node_visuals(self.node_id)

class RemoveChoiceCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        index: int,
        choice_tuple: Tuple[str, str, str],
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Remove Choice from '{node_id}'", parent)
        self.node_id = node_id
        self.index = index
        self.choice_tuple = choice_tuple
        self.restored_next: Optional[str] = None

    def redo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if not node_data:
            logging.warning(
                f"Source node '{self.node_id}' not found for RemoveChoice redo. Obsolete."
            )
            self.setObsolete(True)
            return

        self.restored_next = (
            node_data.next_node
        )  
        removed = False
        try:
            if not isinstance(node_data.choices, list):
                node_data.choices = []

            if (
                0 <= self.index < len(node_data.choices)
                and node_data.choices[self.index] == self.choice_tuple
            ):
                node_data.choices.pop(self.index)
                removed = True
            elif self.choice_tuple in node_data.choices:
                logging.warning(
                    f"RemoveChoice redo: Index mismatch for {self.node_id}[{self.index}], removing by value."
                )
                node_data.choices.remove(self.choice_tuple)
                removed = True

            if removed:
                self.editor.update_properties_panel()
                gnode = self.editor.graphics_nodes.get(self.node_id)
                if gnode:
                    self.editor.draw_edges_for_node(gnode)
                    gnode.update()
                self.editor._mark_unsaved()
                self.editor.update_dynamic_node_visuals(self.node_id)
            else:
                logging.warning(
                    f"Choice {self.choice_tuple} not found at index {self.index} on node {self.node_id}. RemoveChoice obsolete."
                )
                self.setObsolete(True)

        except (ValueError, IndexError) as e:
            logging.exception(f"Error during RemoveChoice redo for {self.node_id}: {e}")
            self.setObsolete(True)

    def undo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if not node_data:
            logging.warning(
                f"Source node '{self.node_id}' not found for RemoveChoice undo. Obsolete."
            )
            self.setObsolete(True)
            return

        if node_data.next_node:
            node_data.next_node = None  
        if not isinstance(node_data.choices, list):
            node_data.choices = []

        try:
            if 0 <= self.index <= len(node_data.choices):
                node_data.choices.insert(self.index, self.choice_tuple)
            else:
                logging.warning(
                    f"Invalid index {self.index} for RemoveChoice undo. Appending."
                )
                node_data.choices.append(self.choice_tuple)
        except Exception as e:
            logging.exception(
                f"Error during RemoveChoice undo insert for {self.node_id}: {e}"
            )
            node_data.choices.append(self.choice_tuple)  

        self.editor.update_properties_panel()
        gnode = self.editor.graphics_nodes.get(self.node_id)
        if gnode:
            self.editor.draw_edges_for_node(gnode)
            gnode.update()
        self.editor._mark_unsaved()
        self.editor.update_dynamic_node_visuals(self.node_id)

class EditChoiceCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        index: int,
        old_choice: Tuple[str, str, str],
        new_choice: Tuple[str, str, str],
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Edit Choice in '{node_id}'", parent)
        self.node_id = node_id
        self.index = index
        self.old_choice = old_choice
        self.new_choice = new_choice

    def _set_choice(self, choice_to_set: Tuple[str, str, str]) -> bool:
        node_data = self.editor.nodes_data.get(self.node_id)
        if not node_data or not isinstance(node_data.choices, list):
            logging.warning(
                f"Node '{self.node_id}' not found or choices invalid for EditChoice. Obsolete."
            )
            self.setObsolete(True)
            return False

        try:
            if 0 <= self.index < len(node_data.choices):

                expected_prior = (
                    self.old_choice
                    if choice_to_set == self.new_choice
                    else self.new_choice
                )
                if node_data.choices[self.index] == expected_prior:
                    node_data.choices[self.index] = choice_to_set
                    self.editor.update_properties_panel()
                    gnode = self.editor.graphics_nodes.get(self.node_id)
                    if gnode:
                        self.editor.draw_edges_for_node(
                            gnode
                        )  
                    self.editor._mark_unsaved()
                    self.editor.update_dynamic_node_visuals(self.node_id)
                    return True
                else:
                    logging.warning(
                        f"EditChoice: Choice at index {self.index} on node {self.node_id} mismatch. Expected '{expected_prior}', found '{node_data.choices[self.index]}'. Obsolete."
                    )
                    self.setObsolete(True)
                    return False
            else:
                logging.warning(
                    f"EditChoice: Invalid index {self.index} for choices list (len={len(node_data.choices)}) on node {self.node_id}. Obsolete."
                )
                self.setObsolete(True)
                return False
        except Exception as e:
            logging.exception(
                f"Error during EditChoice._set_choice for {self.node_id}: {e}"
            )
            self.setObsolete(True)
            return False

    def redo(self):
        self._set_choice(self.new_choice)

    def undo(self):
        self._set_choice(self.old_choice)

class ConvertNextToChoiceCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        old_next_target: str,
        new_choice_text: str,
        new_preset: str,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Convert Next to Choice for '{node_id}'", parent)
        self.node_id = node_id
        self.old_next_target = old_next_target
        self.new_choice_tuple = (
            str(new_choice_text),
            str(old_next_target),
            str(new_preset),
        )

    def redo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if not node_data:
            logging.warning(
                f"Node '{self.node_id}' not found for ConvertNextToChoice redo. Obsolete."
            )
            self.setObsolete(True)
            return
        if node_data.next_node != self.old_next_target:
            logging.warning(
                f"ConvertNextToChoice redo state mismatch for node '{self.node_id}'. Expected next='{self.old_next_target}', found '{node_data.next_node}'. Obsolete."
            )
            self.setObsolete(True)
            return

        node_data.next_node = None
        if not isinstance(node_data.choices, list):
            node_data.choices = []
        node_data.choices.append(self.new_choice_tuple)

        self.editor.update_properties_panel()
        gnode = self.editor.graphics_nodes.get(self.node_id)
        if gnode:
            self.editor.draw_edges_for_node(gnode)
            gnode.update()
        self.editor._mark_unsaved()
        self.editor.update_dynamic_node_visuals(self.node_id)
        logging.debug(
            f"ConvertNextToChoice redo: Converted next->{self.old_next_target} to choice {self.new_choice_tuple} for node {self.node_id}"
        )

    def undo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if not node_data:
            logging.warning(
                f"Node '{self.node_id}' not found for ConvertNextToChoice undo. Obsolete."
            )
            self.setObsolete(True)
            return
        if (
            node_data.next_node is not None
            or self.new_choice_tuple not in node_data.choices
        ):
            logging.warning(
                f"ConvertNextToChoice undo state mismatch for node '{self.node_id}'. Expected next=None and choice={self.new_choice_tuple}, found next='{node_data.next_node}' and choices={node_data.choices}. Obsolete."
            )
            self.setObsolete(True)
            return

        node_data.next_node = self.old_next_target
        try:
            node_data.choices.remove(self.new_choice_tuple)
        except ValueError:
            logging.warning(
                f"Failed to remove choice {self.new_choice_tuple} during ConvertNextToChoice undo for node {self.node_id}."
            )
            node_data.choices = []  

        self.editor.update_properties_panel()
        gnode = self.editor.graphics_nodes.get(self.node_id)
        if gnode:
            self.editor.draw_edges_for_node(gnode)
            gnode.update()
        self.editor._mark_unsaved()
        self.editor.update_dynamic_node_visuals(self.node_id)
        logging.debug(
            f"ConvertNextToChoice undo: Restored next->{self.old_next_target} for node {self.node_id}"
        )

class SetNextNodeCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        target_id: Optional[str],
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        target_desc = f"'{target_id}'" if target_id else "None"
        super().__init__(editor, f"Set Next '{node_id}' -> {target_desc}", parent)
        self.node_id = node_id
        self.new_target_id = target_id
        self.old_target_id: Optional[str] = None
        self.old_choices: List[Tuple[str, str, str]] = []

    def redo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if node_data:
            self.old_target_id = node_data.next_node
            self.old_choices = list(node_data.choices)
            node_data.next_node = self.new_target_id
            node_data.choices = []  

            self.editor.update_properties_panel()
            gnode = self.editor.graphics_nodes.get(self.node_id)
            if gnode:
                self.editor.draw_edges_for_node(gnode)
                gnode.update()
            self.editor._mark_unsaved()
            self.editor.update_dynamic_node_visuals(self.node_id)
        else:
            logging.warning(
                f"Node '{self.node_id}' not found for SetNextNode redo. Obsolete."
            )
            self.setObsolete(True)

    def undo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if node_data:
            node_data.next_node = self.old_target_id
            node_data.choices = list(self.old_choices)

            self.editor.update_properties_panel()
            gnode = self.editor.graphics_nodes.get(self.node_id)
            if gnode:
                self.editor.draw_edges_for_node(gnode)
                gnode.update()
            self.editor._mark_unsaved()
            self.editor.update_dynamic_node_visuals(self.node_id)
        else:
            logging.warning(
                f"Node '{self.node_id}' not found for SetNextNode undo. Obsolete."
            )
            self.setObsolete(True)

class ClearConnectionsCommand(BaseUndoCommand):
    def __init__(
        self,
        node_id: str,
        old_next: Optional[str],
        old_choices: List[Tuple[str, str, str]],
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Clear Connections '{node_id}'", parent)
        self.node_id = node_id
        self.old_next = old_next
        self.old_choices = list(old_choices)

    def redo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if node_data:

            self.old_next = node_data.next_node
            self.old_choices = list(node_data.choices)

            node_data.next_node = None
            node_data.choices = []

            self.editor.update_properties_panel()
            gnode = self.editor.graphics_nodes.get(self.node_id)
            if gnode:
                self.editor.draw_edges_for_node(gnode)
                gnode.update()
            self.editor._mark_unsaved()
            self.editor.update_dynamic_node_visuals(self.node_id)
        else:
            logging.warning(
                f"Node '{self.node_id}' not found for ClearConnections redo. Obsolete."
            )
            self.setObsolete(True)

    def undo(self):
        node_data = self.editor.nodes_data.get(self.node_id)
        if node_data:
            node_data.next_node = self.old_next
            node_data.choices = list(self.old_choices)

            self.editor.update_properties_panel()
            gnode = self.editor.graphics_nodes.get(self.node_id)
            if gnode:
                self.editor.draw_edges_for_node(gnode)
                gnode.update()
            self.editor._mark_unsaved()
            self.editor.update_dynamic_node_visuals(self.node_id)
        else:
            logging.warning(
                f"Node '{self.node_id}' not found for ClearConnections undo. Obsolete."
            )
            self.setObsolete(True)

class RenameNodeIdCommand(BaseUndoCommand):
    def __init__(
        self,
        old_id: str,
        new_id: str,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        super().__init__(editor, f"Rename '{old_id}' -> '{new_id}'", parent)
        self.old_id = old_id
        self.new_id = new_id

    def redo(self):
        gnode = self.editor.graphics_nodes.get(self.old_id)
        if not gnode:
            logging.warning(
                f"Rename redo error: Source node '{self.old_id}' not found. Obsolete."
            )
            self.setObsolete(True)
            return

        if not self.editor._perform_node_rename_internal(
            gnode, self.old_id, self.new_id
        ):
            logging.error(
                f"Rename redo error: Internal rename failed for '{self.old_id}' -> '{self.new_id}'. Obsolete."
            )
            self.setObsolete(True)
        else:
            self.editor._mark_unsaved()

            renamed_node = self.editor.graphics_nodes.get(self.new_id)
            current_selection = self.editor.get_selected_node()
            if (
                renamed_node
                and current_selection
                and current_selection.node_data.id == self.new_id
            ):
                self.editor.update_properties_panel()
            elif renamed_node:  
                renamed_node.update()

    def undo(self):
        gnode = self.editor.graphics_nodes.get(self.new_id)
        if not gnode:
            logging.warning(
                f"Rename undo error: Renamed node '{self.new_id}' not found. Obsolete."
            )
            self.setObsolete(True)
            return

        if not self.editor._perform_node_rename_internal(
            gnode, self.new_id, self.old_id
        ):
            logging.error(
                f"Rename undo error: Internal rename failed for '{self.new_id}' -> '{self.old_id}'. Obsolete."
            )
            self.setObsolete(True)
        else:
            self.editor._mark_unsaved()

            reverted_node = self.editor.graphics_nodes.get(self.old_id)
            current_selection = self.editor.get_selected_node()
            if (
                reverted_node
                and current_selection
                and current_selection.node_data.id == self.old_id
            ):
                self.editor.update_properties_panel()
            elif reverted_node:  
                reverted_node.update()

class DeleteEdgeCommand(BaseUndoCommand):
    def __init__(
        self,
        edge: GraphicsEdge,
        editor: "DialogueEditor",
        parent: Optional[QObject] = None,
    ):
        src_id = (
            edge.source.node_data.id
            if edge.source and hasattr(edge.source, "node_data")
            else "N/A"
        )
        dest_id = (
            edge.dest.node_data.id
            if edge.dest and hasattr(edge.dest, "node_data")
            else "N/A"
        )
        super().__init__(editor, f"Delete Edge {src_id}->{dest_id}", parent)
        self.source_id = src_id
        self.dest_id = dest_id
        self.edge_choice_text = edge.choice_text  
        self.connection_type: Optional[str] = None  
        self.choice_index: int = -1
        self.choice_data: Optional[Tuple] = None

    def redo(self):
        edge = (
            self._find_edge_object()
        )  
        source_node_data = self.editor.nodes_data.get(self.source_id)

        if not source_node_data:
            logging.warning(
                f"Delete edge redo error: Source node '{self.source_id}' not found. Obsolete."
            )
            self.setObsolete(True)
            return

        conn_removed = False

        if source_node_data.next_node == self.dest_id:
            self.connection_type = "next"
            self.choice_index = -1
            self.choice_data = None
            source_node_data.next_node = None
            conn_removed = True
            logging.debug(
                f"DeleteEdge redo: Removing 'next' link {self.source_id}->{self.dest_id}"
            )

        elif isinstance(source_node_data.choices, list):
            found_idx = -1
            found_choice = None
            for i, choice in enumerate(list(source_node_data.choices)):  

                is_match = (choice[1] == self.dest_id) and (
                    self.edge_choice_text is None or choice[0] == self.edge_choice_text
                )
                if is_match:
                    found_idx = i
                    found_choice = choice
                    break  

            if found_idx != -1 and found_choice is not None:
                self.connection_type = "choice"
                self.choice_index = found_idx
                self.choice_data = found_choice
                try:
                    source_node_data.choices.pop(found_idx)
                    conn_removed = True
                    logging.debug(
                        f"DeleteEdge redo: Removing choice {self.choice_data} at index {self.choice_index} from {self.source_id}"
                    )
                except IndexError:
                    logging.error(
                        f"Index error removing choice {found_idx} for edge {self.source_id}->{self.dest_id}."
                    )
                    conn_removed = False  

        if not conn_removed:
            logging.warning(
                f"Data connection {self.source_id}->{self.dest_id} (choice: {self.edge_choice_text}) not found during edge delete redo."
            )

        if edge:
            self.editor.remove_edge_object(edge)  
            logging.debug(f"DeleteEdge redo: Removed graphical edge object.")
        else:
            logging.warning(
                f"DeleteEdge redo: Graphical edge object not found for {self.source_id}->{self.dest_id}."
            )

        self.editor._mark_unsaved()
        self.editor.update_properties_panel()
        gnode = self.editor.graphics_nodes.get(self.source_id)
        if gnode:
            gnode.update()  

    def undo(self):
        source_node = self.editor.graphics_nodes.get(self.source_id)
        dest_node = self.editor.graphics_nodes.get(self.dest_id)
        source_node_data = self.editor.nodes_data.get(self.source_id)

        if not source_node or not dest_node or not source_node_data:
            logging.warning(
                f"Delete edge undo error: Source '{self.source_id}' or Dest '{self.dest_id}' node/data missing. Obsolete."
            )
            self.setObsolete(True)
            return

        if self.connection_type is None:
            logging.error(
                f"Delete edge undo error: No connection type stored for {self.source_id}->{self.dest_id}. Likely redo failed. Obsolete."
            )
            self.setObsolete(True)
            return

        restored = False
        try:
            if self.connection_type == "next":
                source_node_data.next_node = self.dest_id
                source_node_data.choices = []  
                restored = True
                logging.debug(
                    f"DeleteEdge undo: Restoring 'next' link {self.source_id}->{self.dest_id}"
                )
            elif self.connection_type == "choice" and self.choice_data:
                source_node_data.next_node = None  
                if not isinstance(source_node_data.choices, list):
                    source_node_data.choices = []

                if 0 <= self.choice_index <= len(source_node_data.choices):
                    source_node_data.choices.insert(self.choice_index, self.choice_data)
                else:  
                    logging.warning(
                        f"Invalid index {self.choice_index} for undoing choice add. Appending instead."
                    )
                    source_node_data.choices.append(self.choice_data)
                restored = True
                logging.debug(
                    f"DeleteEdge undo: Restoring choice {self.choice_data} at index {self.choice_index} to {self.source_id}"
                )
            else:
                logging.error(
                    f"Delete edge undo error: Invalid connection_type '{self.connection_type}' or missing choice_data."
                )

        except Exception as e:
            logging.exception(
                f"Error restoring data connection during DeleteEdge undo: {e}"
            )
            restored = False

        if restored:

            self.editor.draw_edges_for_node(source_node)
            self.editor._mark_unsaved()
            self.editor.update_properties_panel()
            gnode = self.editor.graphics_nodes.get(self.source_id)
            if gnode:
                gnode.update()
        else:
            self.setObsolete(True)

    def _find_edge_object(self) -> Optional[GraphicsEdge]:
        """Finds the specific GraphicsEdge object matching the command's details."""
        for edge in self.editor.graphics_edges:
            if (
                edge.source
                and edge.dest
                and hasattr(edge.source, "node_data")
                and hasattr(edge.dest, "node_data")
                and edge.source.node_data.id == self.source_id
                and edge.dest.node_data.id == self.dest_id
                and edge.choice_text == self.edge_choice_text
            ):  
                return edge
        return None

class ToggleBookmarkCommand(BaseUndoCommand):
    def __init__(
        self, node_id: str, editor: "DialogueEditor", parent: Optional[QObject] = None
    ):
        self.is_adding = node_id not in editor.bookmarks
        action_text = "Add" if self.is_adding else "Remove"
        super().__init__(editor, f"{action_text} Bookmark for '{node_id}'", parent)
        self.node_id = node_id

    def redo(self):
        if self.is_adding:
            self.editor.bookmarks.add(self.node_id)
        else:
            self.editor.bookmarks.discard(self.node_id)
        self.editor.update_bookmark_menu()
        self.editor.update_dynamic_node_visuals(self.node_id)
        self.editor._mark_unsaved()

    def undo(self):
        if self.is_adding:
            self.editor.bookmarks.discard(self.node_id)
        else:
            self.editor.bookmarks.add(self.node_id)
        self.editor.update_bookmark_menu()
        self.editor.update_dynamic_node_visuals(self.node_id)
        self.editor._mark_unsaved()

class DialogueEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.nodes_data: Dict[str, DialogueNodeData] = {}
        self.graphics_nodes: Dict[str, GraphicsNode] = {}
        self.graphics_edges: List[GraphicsEdge] = []
        self.bookmarks: Set[str] = set()  
        self.clipboard: List[Dict] = []  
        self.choices_list: Optional[ClickAwareListWidget] = None
        self.undo_stack = QUndoStack(self)
        self.start_node_id: Optional[str] = None
        self._text_edit_timer = QTimer()
        self._text_edit_timer.setSingleShot(True)
        self._text_edit_timer.setInterval(config.TEXT_EDIT_SAVE_DELAY_MS)
        self._text_edit_timer.timeout.connect(self._push_text_edit_command)
        self._pending_text_edit: Optional[Tuple[str, str, str]] = None
        self._char_edit_timer = QTimer()
        self._char_edit_timer.setSingleShot(True)
        self._char_edit_timer.setInterval(config.TEXT_EDIT_SAVE_DELAY_MS)
        self._char_edit_timer.timeout.connect(self._push_char_edit_command)
        self._pending_char_edit: Optional[Tuple[str, str, str]] = None
        self._original_node_id: Optional[str] = None
        self._original_node_char: Optional[str] = None
        self._original_node_text: Optional[str] = None
        self._original_custom_props: Dict[str, str] = {}
        self.next_node_id_counter = 1
        self.current_project_path: Optional[str] = None
        self.unsaved_changes: bool = False
        self.is_drawing_edge: bool = False
        self.edge_drag_source_node: Optional[GraphicsNode] = None
        self.edge_drag_line: Optional[QGraphicsLineItem] = None
        self.props_widget: Optional[QWidget] = None
        self.custom_props_group: Optional[QGroupBox] = None
        self.custom_props_form_layout: Optional[QFormLayout] = None
        self._custom_prop_edits: Dict[str, QLineEdit] = {}
        self.node_id_edit: Optional[QLineEdit] = None
        self.node_char_edit: Optional[QLineEdit] = None
        self.node_text_edit: Optional[QTextEdit] = None
        self.add_choice_button: Optional[QPushButton] = None
        self.set_next_button: Optional[QPushButton] = None
        self.edit_choice_button: Optional[QPushButton] = None
        self.clear_connections_button: Optional[QPushButton] = None
        self.scene: Optional[QGraphicsScene] = None
        self.view: Optional[ZoomPanGraphicsView] = None
        self.undo_action: Optional[QAction] = None
        self.redo_action: Optional[QAction] = None
        self.bookmarks_menu: Optional[QMenu] = None
        self._find_results: List[str] = []
        self._find_index: int = -1
        self._find_term: str = ""
        self._find_type: str = ""
        self.setup_ui()
        self._update_window_title()
        logging.info("Dialogue Editor initialized.")

    def _update_window_title(self):
        base_title = config.APP_NAME
        project_name = (
            os.path.basename(self.current_project_path)
            if self.current_project_path
            else "Untitled"
        )
        saved_marker = "*" if self.unsaved_changes else ""
        self.setWindowTitle(f"{base_title} - {project_name}{saved_marker}")

    def _mark_unsaved(self, changed: bool = True):
        if changed != self.unsaved_changes:
            self.unsaved_changes = changed
            self._update_window_title()
            if self.undo_action:
                self.undo_action.setEnabled(self.undo_stack.canUndo())
            if self.redo_action:
                self.redo_action.setEnabled(self.undo_stack.canRedo())
            logging.debug(f"Unsaved changes status set to: {self.unsaved_changes}")

    def _on_char_changed_in_panel(self):
        selected_node = self.get_selected_node()
        if not selected_node or self._original_node_char is None:
            if self._char_edit_timer.isActive():
                self._char_edit_timer.stop()
            self._pending_char_edit = None
            return

        current_char_in_panel = self.node_char_edit.text()

        if selected_node.node_data.character != current_char_in_panel:
            selected_node.node_data.character = current_char_in_panel
            selected_node.update()
            self.update_dynamic_node_visuals(
                selected_node.node_data.id
            )  
            self._mark_unsaved()

        original_char_for_command = self._original_node_char
        if current_char_in_panel == original_char_for_command:
            if self._char_edit_timer.isActive():
                self._char_edit_timer.stop()
                self._pending_char_edit = None
            return

        stored_old_char = original_char_for_command
        if self._pending_char_edit:
            _, stored_old_char, _ = self._pending_char_edit
        self._pending_char_edit = (
            selected_node.node_data.id,
            stored_old_char,
            current_char_in_panel,
        )
        self._char_edit_timer.start()

    def _push_char_edit_command(self):
        self._char_edit_timer.stop()
        if self._pending_char_edit:
            node_id, old_char, new_char = self._pending_char_edit
            self._pending_char_edit = None
            if node_id in self.nodes_data and old_char != new_char:
                cmd = SetNodeCharCommand(node_id, old_char, new_char, self)
                self.undo_stack.push(cmd)
                logging.debug(f"Pushed SetNodeCharCommand for {node_id}.")
                current_selection = self.get_selected_node()
                if current_selection and current_selection.node_data.id == node_id:
                    self._original_node_char = new_char

    def _on_text_changed_in_panel(self):
        selected_node = self.get_selected_node()
        if not selected_node or self._original_node_text is None:
            if self._text_edit_timer.isActive():
                self._text_edit_timer.stop()
            self._pending_text_edit = None
            return

        current_text_in_panel = self.node_text_edit.toPlainText()
        if selected_node.node_data.text != current_text_in_panel:
            selected_node.node_data.text = current_text_in_panel
            selected_node.update()
            self._mark_unsaved()

        original_text_for_command = self._original_node_text
        if current_text_in_panel == original_text_for_command:
            if self._text_edit_timer.isActive():
                self._text_edit_timer.stop()
                self._pending_text_edit = None
            return

        stored_old_text = original_text_for_command
        if self._pending_text_edit:
            _, stored_old_text, _ = self._pending_text_edit
        self._pending_text_edit = (
            selected_node.node_data.id,
            stored_old_text,
            current_text_in_panel,
        )
        self._text_edit_timer.start()

    def _push_text_edit_command(self):
        self._text_edit_timer.stop()
        if self._pending_text_edit:
            node_id, old_text, new_text = self._pending_text_edit
            self._pending_text_edit = None
            if node_id in self.nodes_data and old_text != new_text:
                cmd = SetNodeTextCommand(node_id, old_text, new_text, self)
                self.undo_stack.push(cmd)
                logging.debug(f"Pushed SetNodeTextCommand for {node_id}.")
                current_selection = self.get_selected_node()
                if current_selection and current_selection.node_data.id == node_id:
                    self._original_node_text = new_text

    def closeEvent(self, event: QCloseEvent):
        logging.debug("Close event triggered.")
        if hasattr(self, "undo_stack") and self.undo_stack:
            try:
                self.undo_stack.indexChanged.disconnect(self._handle_undo_stack_change)
                logging.debug("Disconnected undo_stack.indexChanged signal.")
            except TypeError:
                logging.debug(
                    "Undo stack signal handler was already disconnected or not found."
                )
            except RuntimeError:
                logging.debug(
                    "Undo stack C++ object likely deleted before signal disconnection attempt."
                )
            except Exception as e:
                logging.warning(
                    f"Unexpected error disconnecting undo stack signal: {e}"
                )

        try:
            self._push_text_edit_command()
            self._push_char_edit_command()
            self._check_pending_custom_prop_changes()

            if self.unsaved_changes:
                reply = QMessageBox.question(
                    self,
                    "Unsaved Changes",
                    "You have unsaved changes. Save before closing?",
                    QMessageBox.StandardButton.Save
                    | QMessageBox.StandardButton.Discard
                    | QMessageBox.StandardButton.Cancel,
                    QMessageBox.StandardButton.Save,
                )
                if reply == QMessageBox.StandardButton.Save:
                    if self.save_project():
                        event.accept()
                    else:
                        event.ignore()
                elif reply == QMessageBox.StandardButton.Cancel:
                    event.ignore()
                else:
                    event.accept()
            else:
                event.accept()

        except RuntimeError as e:
            logging.error(
                f"RuntimeError during closeEvent processing (after disconnect): {e}"
            )
            event.accept()  
        except Exception as e:
            logging.error(f"Unexpected error during closeEvent processing: {e}")
            event.accept()  

    def find_node(self):
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        search_types = ["Node ID", "Character", "Text"]
        search_type, ok1 = QInputDialog.getItem(
            self, "Find Node", "Search by:", search_types, 0, False
        )
        if not ok1:
            return
        search_term, ok2 = QInputDialog.getText(
            self, "Find Node", f"Enter {search_type} to find:"
        )
        if not ok2 or not search_term.strip():
            return

        self.clear_all_highlights()  

        search_term_lower = search_term.strip().lower()
        self._find_results = []
        self._find_index = -1
        self._find_term = search_term
        self._find_type = search_type

        sorted_node_ids = sorted(self.nodes_data.keys())
        for node_id in sorted_node_ids:
            node_data = self.nodes_data.get(node_id)
            if not node_data:
                continue
            match = False
            try:
                if search_type == "Node ID":
                    match = node_id.lower() == search_term_lower
                elif search_type == "Character":
                    match = search_term_lower in node_data.character.lower()
                elif search_type == "Text":
                    match = search_term_lower in node_data.text.lower()
            except AttributeError:
                logging.warning(f"Attribute error searching node {node_id}. Skipping.")
                continue
            if match:
                self._find_results.append(node_id)

        if self._find_results:
            self.find_next_node()  

            for node_id in self._find_results:
                gnode = self.graphics_nodes.get(node_id)
                if gnode:
                    gnode.set_find_highlight(True)

                edges = self.get_edges_for_node(gnode) if gnode else []
                for edge in edges:
                    edge.set_find_highlight(True)

            QMessageBox.information(
                self,
                "Find Results",
                f"Found {len(self._find_results)} matching node(s).\nUse 'Find Next' (Ctrl+G) to cycle.",
            )
        else:
            QMessageBox.information(
                self,
                "Not Found",
                f"No node found matching '{search_term}' for '{search_type}'.",
            )

    def find_next_node(self):
        if not self._find_results:
            self.find_node()  
            return
        self._find_index = (self._find_index + 1) % len(self._find_results)
        found_node_id = self._find_results[self._find_index]
        self.handle_focus_request(found_node_id)
        self.statusBar().showMessage(
            f"Find result {self._find_index + 1}/{len(self._find_results)}: '{found_node_id}'",
            3000,
        )

    def _handle_undo_stack_change(self, idx):
        if hasattr(self, "undo_stack") and self.undo_stack:
            try:
                is_clean = self.undo_stack.isClean()
                self._mark_unsaved(not is_clean)
            except RuntimeError as e:
                logging.warning(
                    f"Caught RuntimeError accessing undo_stack.isClean() in handler: {e}"
                )
            except Exception as e:
                logging.warning(
                    f"Unexpected error accessing undo_stack.isClean() in handler: {e}"
                )
        else:
            logging.debug(
                "_handle_undo_stack_change called but self.undo_stack is missing/None."
            )

    def setup_ui(self):
        self.setWindowTitle(config.APP_NAME)
        self.setGeometry(100, 100, 1200, 700)
        self.statusBar()  

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        self.scene = QGraphicsScene()
        self.scene.setBackgroundBrush(config.SCENE_BACKGROUND_COLOR)
        self.view = ZoomPanGraphicsView(self.scene, self, self)
        splitter.addWidget(self.view)

        self.props_widget = QWidget()
        props_layout = QVBoxLayout(self.props_widget)
        props_layout.setContentsMargins(5, 5, 5, 5)
        self.props_widget.setFixedWidth(350)
        splitter.addWidget(self.props_widget)

        initial_width = self.width()
        splitter.setSizes([int(initial_width * 0.7), int(initial_width * 0.3)])

        node_group = QGroupBox("Node Properties")
        node_form = QFormLayout()
        self.node_id_edit = QLineEdit()
        self.node_id_edit.editingFinished.connect(self.handle_node_id_change)
        self.node_char_edit = QLineEdit()
        self.node_char_edit.textChanged.connect(self._on_char_changed_in_panel)
        self.node_text_edit = QTextEdit()
        self.node_text_edit.setAcceptRichText(False)
        self.node_text_edit.setMinimumHeight(100)
        self.node_text_edit.textChanged.connect(self._on_text_changed_in_panel)
        node_form.addRow("ID:", self.node_id_edit)
        node_form.addRow("Character:", self.node_char_edit)
        node_form.addRow("Text:", self.node_text_edit)
        node_group.setLayout(node_form)
        props_layout.addWidget(node_group)

        self.custom_props_group = QGroupBox("Custom Properties")
        self.custom_props_form_layout = QFormLayout()
        self.custom_props_group.setLayout(self.custom_props_form_layout)
        props_layout.addWidget(self.custom_props_group)

        choices_group = QGroupBox("Choices / Next / Loop")
        choices_layout = QVBoxLayout()
        self.choices_list = ClickAwareListWidget(self, self)
        self.add_choice_button = QPushButton("Add Choice...")
        self.set_next_button = QPushButton("Set 'Next' Node...")
        self.edit_choice_button = QPushButton("Edit Choice")
        self.edit_choice_button.setEnabled(False)
        self.clear_connections_button = QPushButton("Clear All Connections")

        if self.choices_list is not None:
            self.choices_list.focusRequested.connect(self.handle_focus_request)
            self.choices_list.editRequested.connect(self.edit_choice_item_cmd)
            self.choices_list.nextToChoiceRequested.connect(
                self.handle_next_to_choice_request
            )
            self.choices_list.setContextMenuPolicy(
                Qt.ContextMenuPolicy.CustomContextMenu
            )
            self.choices_list.customContextMenuRequested.connect(
                self.show_choices_context_menu
            )
            self.choices_list.currentItemChanged.connect(
                self.handle_current_choice_item_changed
            )

            choice_buttons_layout = QHBoxLayout()
            choice_buttons_layout.addWidget(self.add_choice_button)
            choice_buttons_layout.addWidget(self.set_next_button)
            choice_buttons_layout.addWidget(self.edit_choice_button)

            choices_layout.addWidget(
                QLabel(
                    "Connections:\n- Double-Click Choice/Next to Edit.\n- Click Target ID Area to Focus Node."
                )
            )  
            choices_layout.addWidget(self.choices_list)
            choices_layout.addLayout(choice_buttons_layout)
            choices_layout.addWidget(self.clear_connections_button)

            self.add_choice_button.clicked.connect(self.add_choice_dialog)
            self.set_next_button.clicked.connect(self.set_next_node_dialog)
            self.edit_choice_button.clicked.connect(
                self.handle_edit_choice_button_clicked
            )
            self.clear_connections_button.clicked.connect(self.clear_connections_cmd)
        else:
            error_label = QLabel("Error: Choices list failed to load.")
            choices_layout.addWidget(error_label)

        choices_group.setLayout(choices_layout)
        props_layout.addWidget(choices_group)
        props_layout.addStretch()

        self._setup_menubar()

        if self.scene:
            self.scene.selectionChanged.connect(self.update_properties_panel)
        if self.view:
            self.view.edgeDragStarted.connect(self.handle_edge_drag_started)
            self.view.edgeDragMoved.connect(self.handle_edge_drag_moved)
            self.view.edgeDragEnded.connect(self.handle_edge_drag_ended)
            self.view.edgeDragCancelled.connect(self.handle_edge_drag_cancelled)

        if hasattr(self, "undo_action") and self.undo_action:
            self.undo_stack.canUndoChanged.connect(self.undo_action.setEnabled)
        if hasattr(self, "redo_action") and self.redo_action:
            self.undo_stack.canRedoChanged.connect(self.redo_action.setEnabled)
        if hasattr(self, "undo_stack"):
            self.undo_stack.indexChanged.connect(self._handle_undo_stack_change)

        self.update_properties_panel()

    def _setup_menubar(self):
        menubar = self.menuBar()

        file_menu = menubar.addMenu("&File")
        new_action = QAction("&New Project", self)
        new_action.triggered.connect(self.new_file)
        file_menu.addAction(new_action)
        open_action = QAction("&Open Project...", self)
        open_action.triggered.connect(self.load_project)
        file_menu.addAction(open_action)
        save_action = QAction("&Save Project", self)
        save_action.setShortcut(QKeySequence.StandardKey.Save)
        save_action.triggered.connect(self.save_project)
        file_menu.addAction(save_action)
        save_as_action = QAction("Save Project &As...", self)
        save_as_action.setShortcut(QKeySequence.StandardKey.SaveAs)
        save_as_action.triggered.connect(self.save_project_as)
        file_menu.addAction(save_as_action)
        file_menu.addSeparator()
        import_action = QAction("&Import from JSON...", self)
        import_action.triggered.connect(self.import_from_json)
        file_menu.addAction(import_action)
        export_action = QAction("&Export to JSON...", self)
        export_action.triggered.connect(self.export_to_json)
        file_menu.addAction(export_action)
        file_menu.addSeparator()
        validate_action = QAction("&Validate Dialogue...", self)
        validate_action.triggered.connect(self.validate_dialogue)
        file_menu.addAction(validate_action)
        file_menu.addSeparator()
        about_action = QAction("&About Dialogue Editor", self)
        about_action.triggered.connect(self.show_about_dialog)
        file_menu.addAction(about_action)
        exit_action = QAction("&Exit", self)
        exit_action.setShortcut(QKeySequence.StandardKey.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        edit_menu = menubar.addMenu("&Edit")
        self.undo_action = self.undo_stack.createUndoAction(self, "&Undo")
        self.undo_action.setShortcut(QKeySequence.StandardKey.Undo)
        self.undo_action.setEnabled(False)
        edit_menu.addAction(self.undo_action)
        self.redo_action = self.undo_stack.createRedoAction(self, "&Redo")
        self.redo_action.setShortcut(QKeySequence.StandardKey.Redo)
        self.redo_action.setEnabled(False)
        edit_menu.addAction(self.redo_action)
        edit_menu.addSeparator()
        copy_action = QAction("&Copy", self)
        copy_action.setShortcut(QKeySequence.StandardKey.Copy)
        copy_action.triggered.connect(self.copy_selected_nodes)
        edit_menu.addAction(copy_action)
        paste_action = QAction("&Paste", self)
        paste_action.setShortcut(QKeySequence.StandardKey.Paste)
        paste_action.triggered.connect(lambda: self.paste_nodes())
        edit_menu.addAction(paste_action)
        edit_menu.addSeparator()
        find_action = QAction("&Find Node...", self)
        find_action.setShortcut(QKeySequence.StandardKey.Find)
        find_action.triggered.connect(self.find_node)
        edit_menu.addAction(find_action)
        find_next_action = QAction("Find &Next", self)
        find_next_action.setShortcut(QKeySequence.StandardKey.FindNext)
        find_next_action.triggered.connect(self.find_next_node)
        edit_menu.addAction(find_next_action)
        edit_menu.addSeparator()
        add_node_action = QAction("Add &Node", self)
        add_node_action.setShortcut(QKeySequence.StandardKey.New)
        add_node_action.triggered.connect(
            lambda: self.add_node_cmd(
                pos=(
                    self.view.mapToScene(self.view.viewport().rect().center())
                    if self.view
                    else None
                ),
                character=self.get_default_character_for_new_node(),
            )
        )
        edit_menu.addAction(add_node_action)
        delete_sel_action = QAction("&Delete Selected", self)
        delete_sel_action.setShortcut(QKeySequence.StandardKey.Delete)
        delete_sel_action.triggered.connect(self.delete_selected_items_cmd)
        edit_menu.addAction(delete_sel_action)
        link_sel_action = QAction("&Link Selected Nodes", self)
        link_sel_action.setShortcut("Ctrl+L")
        link_sel_action.triggered.connect(self.link_selected_nodes_cmd)
        edit_menu.addAction(link_sel_action)

        view_menu = menubar.addMenu("&View")
        self.bookmarks_menu = view_menu.addMenu("&Bookmarks")
        view_menu.addSeparator()
        highlight_outgoing_action = QAction("Highlight &Outgoing Path", self)
        highlight_outgoing_action.triggered.connect(
            lambda: self.highlight_outgoing_path()
        )
        view_menu.addAction(highlight_outgoing_action)
        highlight_incoming_action = QAction("Highlight &Incoming Path", self)
        highlight_incoming_action.triggered.connect(
            lambda: self.highlight_incoming_path()
        )
        view_menu.addAction(highlight_incoming_action)
        clear_highlight_action = QAction("&Clear Highlights", self)
        clear_highlight_action.setShortcut(QKeySequence(Qt.Key.Key_Escape))
        clear_highlight_action.triggered.connect(self.clear_all_highlights)
        view_menu.addAction(clear_highlight_action)
        view_menu.addSeparator()
        layout_action = QAction("Apply Auto-&Layout", self)
        layout_action.setShortcut("Ctrl+R")
        layout_action.setToolTip(
            "Arrange nodes automatically based on connections (Sugiyama)"
        )
        layout_action.triggered.connect(self._run_auto_layout)
        view_menu.addAction(layout_action)
        fit_view_action = QAction("&Fit View to Content", self)
        fit_view_action.setShortcut("Ctrl+F")
        fit_view_action.triggered.connect(self._fit_view_after_layout)
        view_menu.addAction(fit_view_action)

        self.update_bookmark_menu()  

    def _get_unique_node_id(self, base: str = "node_") -> str:
        max_existing_num = 0
        prefix_len = len(base)
        for node_id in self.nodes_data.keys():
            if node_id.startswith(base):
                try:
                    num_part_str = node_id[prefix_len:]
                    if num_part_str.isdigit():
                        max_existing_num = max(max_existing_num, int(num_part_str))
                except (ValueError, IndexError):
                    pass
        self.next_node_id_counter = max(self.next_node_id_counter, max_existing_num + 1)
        new_id = f"{base}{self.next_node_id_counter}"
        while new_id in self.nodes_data:
            self.next_node_id_counter += 1
            new_id = f"{base}{self.next_node_id_counter}"
        self.next_node_id_counter += 1
        return new_id

    def _add_node_internal(self, node_data: DialogueNodeData) -> Optional[GraphicsNode]:
        if node_data.id in self.nodes_data:
            logging.error(
                f"Internal Error: Node ID '{node_data.id}' already exists during _add_node_internal."
            )
            return None
        if not isinstance(node_data, DialogueNodeData):
            logging.error(
                f"Internal Error: Invalid data type '{type(node_data)}' provided to _add_node_internal."
            )
            return None

        try:
            graphics_node = GraphicsNode(node_data, self)
            graphics_node.setPos(node_data.pos)
            if self.scene:
                self.scene.addItem(graphics_node)
            else:
                logging.error("Cannot add node graphics: Scene not initialized.")
                return None

            self.nodes_data[node_data.id] = node_data
            self.graphics_nodes[node_data.id] = graphics_node
            logging.info(f"Added node '{node_data.id}' internally.")
            self.update_dynamic_node_visuals(node_data.id)  
            return graphics_node
        except Exception as e:
            logging.exception(
                f"Error creating or adding GraphicsNode instance for '{node_data.id}': {e}"
            )
            if node_data.id in self.nodes_data:
                del self.nodes_data[node_data.id]
            if node_data.id in self.graphics_nodes:
                del self.graphics_nodes[node_data.id]
            return None

    def _delete_node_internal(self, node_id_to_delete: str) -> bool:
        if node_id_to_delete not in self.nodes_data:
            logging.warning(f"Cannot delete non-existent node '{node_id_to_delete}'.")
            return False
        logging.info(f"Deleting node '{node_id_to_delete}' internally...")

        graphics_node = self.graphics_nodes.get(node_id_to_delete)
        edges_to_remove = (
            self.get_edges_for_node(graphics_node) if graphics_node else []
        )
        logging.debug(
            f"Removing {len(edges_to_remove)} edges connected to '{node_id_to_delete}'."
        )
        for edge in list(edges_to_remove):
            self.remove_edge_object(edge)

        if graphics_node and graphics_node.scene() == self.scene:
            self.scene.removeItem(graphics_node)
            logging.debug(f"Removed graphics item for '{node_id_to_delete}'.")
        elif graphics_node:
            logging.warning(
                f"Graphics node '{node_id_to_delete}' found but not in the current scene."
            )

        nodes_to_update = []
        for other_id, other_data in self.nodes_data.items():
            updated = False
            if other_id == node_id_to_delete:
                continue
            if other_data.next_node == node_id_to_delete:
                other_data.next_node = None
                updated = True
            original_choice_count = len(other_data.choices)
            other_data.choices = [
                c for c in other_data.choices if c[1] != node_id_to_delete
            ]
            if len(other_data.choices) < original_choice_count:
                updated = True
            if updated:
                nodes_to_update.append(other_id)
                logging.debug(
                    f"Removed data links from '{other_id}' to deleted node '{node_id_to_delete}'."
                )

        if node_id_to_delete in self.graphics_nodes:
            del self.graphics_nodes[node_id_to_delete]
        if node_id_to_delete in self.nodes_data:
            del self.nodes_data[node_id_to_delete]

        if self.start_node_id == node_id_to_delete:
            old_start = self.start_node_id
            self.start_node_id = None
            logging.info(f"Deleted the start node '{old_start}'.")

            new_start_id: Optional[str] = None
            target_id = config.START_NODE_EXPORT_ID
            if target_id in self.nodes_data:
                new_start_id = target_id
            elif self.nodes_data:
                new_start_id = sorted(self.nodes_data.keys())[0]

            if new_start_id and new_start_id in self.graphics_nodes:
                logging.info(
                    f"Attempting to set fallback start node to '{new_start_id}'."
                )
                if not self._set_start_node_internal(
                    self.graphics_nodes[new_start_id], force=True
                ):
                    logging.warning(
                        f"Failed to set fallback start node '{new_start_id}' after deleting original."
                    )
            elif self.nodes_data:
                logging.warning(
                    "Could not set fallback start node: No graphics item found."
                )
            else:
                logging.info("No remaining nodes to set as start node after deletion.")

        if node_id_to_delete in self.bookmarks:
            self.bookmarks.discard(node_id_to_delete)
            self.update_bookmark_menu()

        logging.info(f"Successfully deleted node '{node_id_to_delete}'.")
        return True

    def _set_start_node_internal(
        self, graphics_node_to_set: GraphicsNode, force: bool = False
    ) -> bool:
        if not graphics_node_to_set or not hasattr(graphics_node_to_set, "node_data"):
            logging.error("Invalid node provided to _set_start_node_internal.")
            return False

        new_start_original_id = graphics_node_to_set.node_data.id
        target_start_id = config.START_NODE_EXPORT_ID

        already_is_start = self.start_node_id == new_start_original_id
        needs_rename = new_start_original_id != target_start_id

        if already_is_start and not needs_rename and not force:
            logging.debug(
                f"Node '{new_start_original_id}' is already the start node. No action needed."
            )
            return True

        logging.info(
            f"Setting start node internally: Target='{new_start_original_id}', Current='{self.start_node_id}', Force={force}"
        )

        node_to_operate_on = graphics_node_to_set
        final_id_after_rename = new_start_original_id

        if needs_rename:
            logging.info(
                f"Start node requires rename: '{new_start_original_id}' -> '{target_start_id}'"
            )
            if not self._handle_start_node_conflict(
                node_to_operate_on, target_start_id
            ):
                return False

            if not self._perform_node_rename_internal(
                node_to_operate_on, new_start_original_id, target_start_id
            ):
                return False
            final_id_after_rename = target_start_id

            node_to_operate_on = self.graphics_nodes.get(target_start_id)
            if not node_to_operate_on:
                logging.critical(
                    f"CRITICAL: Node reference lost after rename to '{target_start_id}'."
                )
                return False

        old_start_node_id = self.start_node_id
        if old_start_node_id and old_start_node_id != final_id_after_rename:
            self._unset_start_node_visuals(old_start_node_id)

        self.start_node_id = final_id_after_rename
        if self.start_node_id in self.nodes_data:
            self.nodes_data[self.start_node_id].is_start_node = True

        if node_to_operate_on and hasattr(node_to_operate_on, "node_data"):
            node_to_operate_on.node_data.is_start_node = True
            node_to_operate_on.update()
        else:
            logging.warning(
                f"Could not update data/visuals for new start node '{self.start_node_id}'."
            )

        logging.info(f"Internal start node set successfully to '{self.start_node_id}'.")
        self.redraw_all_edges()  
        return True

    def _handle_start_node_conflict(
        self, node_to_set: GraphicsNode, target_id: str = "start"
    ) -> bool:
        existing_data = self.nodes_data.get(target_id)
        if (
            existing_data
            and node_to_set
            and hasattr(node_to_set, "node_data")
            and existing_data != node_to_set.node_data
        ):
            QMessageBox.critical(
                self,
                "ID Conflict",
                f"Cannot rename node to '{target_id}' because another node already has that ID.",
            )
            logging.error(
                f"Start node rename conflict: Target ID '{target_id}' already exists."
            )
            return False
        return True

    def _unset_start_node_visuals(self, node_id_to_unset: Optional[str]):
        if not node_id_to_unset:
            return
        logging.debug(f"Unsetting start node visuals/flag for '{node_id_to_unset}'.")
        if node_id_to_unset in self.nodes_data:
            self.nodes_data[node_id_to_unset].is_start_node = False
        gnode = self.graphics_nodes.get(node_id_to_unset)
        if gnode:
            if hasattr(gnode, "node_data"):
                gnode.node_data.is_start_node = False
            gnode.update()

    def _perform_node_rename_internal(
        self, gnode_to_rename: GraphicsNode, old_id: str, new_id: str
    ) -> bool:
        if old_id == new_id:
            return True  

        if not new_id or not isinstance(new_id, str) or " " in new_id.strip():
            logging.error(
                f"Invalid new node ID '{new_id}'. Cannot be empty or contain spaces."
            )
            QMessageBox.warning(
                self, "Invalid ID", "Node ID cannot be empty or contain spaces."
            )
            return False
        new_id = new_id.strip()

        if new_id in self.nodes_data:

            if self.nodes_data[new_id] == gnode_to_rename.node_data:
                logging.warning(
                    f"Node '{old_id}' appears to already have target ID '{new_id}'. Assuming rename already happened."
                )

                if self.graphics_nodes.get(new_id) is not gnode_to_rename:
                    self.graphics_nodes[new_id] = gnode_to_rename
                if (
                    old_id != new_id
                    and old_id in self.graphics_nodes
                    and self.graphics_nodes[old_id] == gnode_to_rename
                ):
                    del self.graphics_nodes[old_id]
                if self.start_node_id == old_id:
                    self.start_node_id = new_id  
                return True
            else:
                logging.error(
                    f"Rename target ID '{new_id}' already exists and belongs to a different node."
                )
                QMessageBox.critical(
                    self, "Rename Failed", f"Node ID '{new_id}' already exists."
                )
                return False

        if old_id not in self.nodes_data or old_id not in self.graphics_nodes:
            logging.error(f"Rename source ID '{old_id}' does not exist.")
            return False
        if gnode_to_rename != self.graphics_nodes.get(old_id):
            logging.critical(
                f"CRITICAL: Mismatch between graphics node provided and node found at source ID '{old_id}'. Aborting rename."
            )
            return False

        logging.info(f"Performing internal rename: '{old_id}' -> '{new_id}'")
        try:

            node_data = self.nodes_data[old_id]
            node_data.id = new_id
            gnode_to_rename.node_data.id = new_id
            self.nodes_data[new_id] = node_data
            self.graphics_nodes[new_id] = gnode_to_rename
            del self.nodes_data[old_id]
            del self.graphics_nodes[old_id]

            references_updated_count = 0
            for other_id, other_data in self.nodes_data.items():
                if other_id == new_id:
                    continue  
                connection_updated = False

                if other_data.next_node == old_id:
                    other_data.next_node = new_id
                    connection_updated = True

                new_choices = []
                choice_list_changed = False
                if isinstance(other_data.choices, list):
                    for i, choice_tuple in enumerate(other_data.choices):
                        try:
                            ctext, tid, cpreset = choice_tuple
                            if tid == old_id:
                                new_choices.append((ctext, new_id, cpreset))
                                choice_list_changed = True
                            else:
                                new_choices.append(choice_tuple)
                        except (TypeError, ValueError, IndexError):
                            logging.warning(
                                f"Skipping invalid choice tuple {choice_tuple} in node {other_id} during rename."
                            )
                            new_choices.append(
                                choice_tuple
                            )  
                    if choice_list_changed:
                        other_data.choices = new_choices
                        connection_updated = True

                if connection_updated:
                    references_updated_count += 1

            logging.debug(
                f"Updated references to '{old_id}' in {references_updated_count} other nodes."
            )

            if self.start_node_id == old_id:
                self.start_node_id = new_id
                logging.debug(
                    f"Updated start node ID reference from '{old_id}' to '{new_id}'."
                )

            if old_id in self.bookmarks:
                self.bookmarks.discard(old_id)
                self.bookmarks.add(new_id)
                self.update_bookmark_menu()

            gnode_to_rename.update()
            self.redraw_all_edges()

            logging.info(f"Rename successful: '{old_id}' -> '{new_id}'")
            return True

        except Exception as e:
            logging.exception(
                f"CRITICAL RENAME ERROR '{old_id}'->'{new_id}': {e}. Attempting basic rollback..."
            )
            try:

                rolled_back_data = self.nodes_data.pop(new_id, None)
                rolled_back_gnode = self.graphics_nodes.pop(new_id, None)
                if rolled_back_data:
                    rolled_back_data.id = old_id
                    self.nodes_data[old_id] = rolled_back_data
                if rolled_back_gnode:
                    if hasattr(rolled_back_gnode, "node_data"):
                        rolled_back_gnode.node_data.id = old_id
                    self.graphics_nodes[old_id] = rolled_back_gnode

                if hasattr(
                    gnode_to_rename, "node_data"
                ) and gnode_to_rename.node_data == self.nodes_data.get(old_id):
                    gnode_to_rename.node_data.id = old_id
                elif old_id in self.nodes_data and self.graphics_nodes.get(old_id):
                    self.graphics_nodes[old_id].node_data.id = old_id  

                if self.start_node_id == new_id:
                    self.start_node_id = old_id

                if new_id in self.bookmarks:
                    self.bookmarks.discard(new_id)
                    self.bookmarks.add(old_id)
                    self.update_bookmark_menu()

                logging.warning(
                    "Basic rollback attempted after rename error. State might be inconsistent, especially connections."
                )
                self.redraw_all_edges()
            except Exception as rb_e:
                logging.exception(
                    f"Rollback during rename error failed: {rb_e}. State likely inconsistent."
                )

            QMessageBox.critical(
                self,
                "Rename Failed",
                f"An internal error occurred during rename:\n{e}\nAttempted rollback, but state may be unstable. Please save and reopen.",
            )
            return False

    def add_node_cmd(
        self,
        node_id: Optional[str] = None,
        character: str = "",
        text: str = "",
        pos: Optional[QPointF] = None,
        is_start: bool = False,
    ):
        final_node_id = node_id
        is_new_id_generated = False

        if node_id is None or not node_id.strip():
            final_node_id = self._get_unique_node_id()
            is_new_id_generated = True
            logging.info(f"No/empty node ID provided, generated: '{final_node_id}'")
        else:
            final_node_id = node_id.strip()
            if final_node_id in self.nodes_data:
                conflict_base = final_node_id
                final_node_id = self._get_unique_node_id(base=final_node_id + "_")
                is_new_id_generated = True
                logging.warning(
                    f"Provided node ID '{conflict_base}' already exists. Generated unique ID: '{final_node_id}'"
                )
                QMessageBox.warning(
                    self,
                    "ID Conflict",
                    f"Node ID '{conflict_base}' already exists.\nUsing generated ID: '{final_node_id}'",
                )
            elif (
                final_node_id == config.START_NODE_EXPORT_ID
                and config.START_NODE_EXPORT_ID in self.nodes_data
            ):
                conflict_base = final_node_id
                final_node_id = self._get_unique_node_id(base=final_node_id + "_")
                is_new_id_generated = True
                logging.warning(
                    f"Cannot add node explicitly named '{conflict_base}'. Generated unique ID: '{final_node_id}'"
                )
                QMessageBox.warning(
                    self,
                    "ID Conflict",
                    f"Node ID '{conflict_base}' is reserved or already exists.\nUsing generated ID: '{final_node_id}'",
                )

        final_pos = pos
        if final_pos is None:
            max_y = 50.0
            center_x = 100.0
            if self.view:
                view_center = self.view.mapToScene(self.view.viewport().rect().center())
                center_x = view_center.x()

            if self.graphics_nodes:
                max_y = 0
                for gnode in self.graphics_nodes.values():
                    max_y = max(max_y, gnode.sceneBoundingRect().bottom())

            node_height = getattr(config, "NODE_HEIGHT", 100)
            v_spacing = getattr(config, "NODE_V_SPACING", 50)
            node_width = getattr(config, "NODE_WIDTH", 200)
            final_pos_y = max(50.0, max_y + v_spacing)
            final_pos_x = center_x - (node_width / 2.0)
            final_pos = QPointF(final_pos_x, final_pos_y)
            logging.debug(f"Calculated node position (below existing): {final_pos}")

        try:
            node_data = DialogueNodeData(
                node_id=final_node_id,
                character=character,
                text=text,
                pos=final_pos,
                is_start=is_start or not self.nodes_data,
            )
        except Exception as e:
            logging.exception(
                f"Failed to create DialogueNodeData for '{final_node_id}': {e}"
            )
            QMessageBox.critical(
                self, "Add Node Failed", f"Error creating node data:\n{e}"
            )
            return

        cmd = AddNodeCommand(node_data, self)
        self.undo_stack.push(cmd)

    def delete_node_cmd(self, graphics_node: GraphicsNode):
        if not graphics_node or not hasattr(graphics_node, "node_data"):
            return
        node_id = graphics_node.node_data.id
        reply = QMessageBox.question(
            self,
            "Confirm Delete",
            f"Are you sure you want to delete node '{node_id}' and all its connections?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._push_text_edit_command()
            self._push_char_edit_command()
            self._check_pending_custom_prop_changes()
            cmd = DeleteNodeCommand(node_id, self)
            self.undo_stack.push(cmd)

    def delete_selected_items_cmd(self):
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        nodes = self.get_selected_nodes()
        edges = self.get_selected_edges()
        groups = self.get_selected_visual_groups()

        if not nodes and not edges and not groups:
            return

        n_ids = [n.node_data.id for n in nodes if hasattr(n, "node_data")]
        e_desc = []
        for e in edges:
            s_id = (
                e.source.node_data.id
                if e.source and hasattr(e.source, "node_data")
                else "?"
            )
            d_id = (
                e.dest.node_data.id if e.dest and hasattr(e.dest, "node_data") else "?"
            )
            e_desc.append(f"{s_id}->{d_id}")
        g_labels = [g.label for g in groups]

        msg = "Delete selected items?\n"
        if n_ids:
            msg += f"\nNodes: {', '.join(n_ids)}"
        if e_desc:
            msg += f"\nConnections: {', '.join(e_desc)}"
        if g_labels:
            msg += f"\nGroups: {', '.join(g_labels)}"

        reply = QMessageBox.question(
            self,
            "Confirm Delete",
            msg,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.No:
            return

        self.undo_stack.beginMacro("Delete Selected Items")
        try:

            for edge in edges:
                if edge in self.graphics_edges:
                    self.undo_stack.push(DeleteEdgeCommand(edge, self))
                else:
                    logging.warning(
                        f"Skipping delete command for edge {e_desc[edges.index(edge)]} as it's no longer in graphics_edges."
                    )

            for node in nodes:
                if node.node_data.id in self.nodes_data:
                    self.undo_stack.push(DeleteNodeCommand(node.node_data.id, self))
                else:
                    logging.warning(
                        f"Skipping delete command for node {node.node_data.id} as it's no longer in nodes_data."
                    )

        finally:
            self.undo_stack.endMacro()

    def set_start_node_cmd(self, graphics_node: GraphicsNode):
        if not graphics_node or not hasattr(graphics_node, "node_data"):
            return
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()

        new_id = graphics_node.node_data.id
        old_id = self.start_node_id
        target_id = config.START_NODE_EXPORT_ID

        if new_id == old_id:
            if new_id != target_id:
                if self._handle_start_node_conflict(graphics_node, target_id):
                    logging.info(
                        f"Node '{new_id}' is start, pushing rename to '{target_id}'."
                    )
                    self.undo_stack.push(RenameNodeIdCommand(new_id, target_id, self))
            return  

        if new_id != target_id:
            if not self._handle_start_node_conflict(graphics_node, target_id):
                logging.warning(
                    f"Cannot set '{new_id}' as start due to rename conflict with '{target_id}'."
                )
                return

        logging.info(f"Pushing SetStartNodeCommand: New='{new_id}', Old='{old_id}'")
        self.undo_stack.push(SetStartNodeCommand(new_id, old_id, self))

    def clear_connections_cmd(self):
        selected_node = self.get_selected_node()
        if not selected_node:
            return
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        data = selected_node.node_data
        if not data.next_node and not data.choices:
            logging.info(
                f"Node '{data.id}' already has no connections. Clear command ignored."
            )
            return
        cmd = ClearConnectionsCommand(data.id, data.next_node, list(data.choices), self)
        self.undo_stack.push(cmd)

    def delete_edge_cmd(self, edge: GraphicsEdge):
        if not edge or not edge.source or not edge.dest:
            logging.warning("Attempting to delete invalid edge object.")
            return
        if edge not in self.graphics_edges:
            logging.warning(f"Attempting to delete edge not tracked: {edge}")
            return
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        cmd = DeleteEdgeCommand(edge, self)
        self.undo_stack.push(cmd)

    def _update_properties_node_fields(self, node_data: DialogueNodeData):
        if not self.node_id_edit or not self.node_char_edit or not self.node_text_edit:
            return
        self.node_id_edit.setText(node_data.id)
        self.node_id_edit.setReadOnly(
            node_data.id == config.START_NODE_EXPORT_ID
        )  
        self.node_char_edit.setText(node_data.character)
        if self.node_text_edit.toPlainText() != node_data.text:
            self.node_text_edit.setPlainText(
                node_data.text
            )  
        self._original_node_id = node_data.id
        self._original_node_char = node_data.character
        self._original_node_text = node_data.text

    def _update_properties_custom_fields(self, node_data: Optional[DialogueNodeData]):
        if self.custom_props_form_layout is None:
            logging.error(
                "Cannot update custom props: custom_props_form_layout is None."
            )
            return

        while self.custom_props_form_layout.rowCount() > 0:
            self.custom_props_form_layout.removeRow(0)
        self._custom_prop_edits.clear()
        self._original_custom_props.clear()

        if node_data is None:
            return  

        for internal_key, prop_config in config.ALLOWED_CUSTOM_PROPERTIES.items():
            display_name = prop_config.get("display", internal_key)
            current_value_str = ""
            if isinstance(node_data.custom_data, dict):
                current_value = node_data.custom_data.get(
                    internal_key, prop_config.get("default", "")
                )
                current_value_str = str(current_value)
            else:  
                logging.warning(
                    f"Node '{node_data.id}' has invalid custom_data type. Resetting."
                )
                node_data.custom_data = {}
                current_value_str = str(prop_config.get("default", ""))

            self._original_custom_props[internal_key] = current_value_str

            label = QLabel(f"{display_name}:")
            line_edit = QLineEdit()
            line_edit.setText(current_value_str)
            line_edit.setProperty("internal_key", internal_key)  
            line_edit.editingFinished.connect(self.handle_defined_custom_prop_change)

            self.custom_props_form_layout.addRow(label, line_edit)
            self._custom_prop_edits[internal_key] = line_edit

    def _update_properties_connections_list(
        self, node_data: Optional[DialogueNodeData]
    ):
        if self.choices_list is None:
            logging.error(
                "CRITICAL: self.choices_list object is None, cannot update list widget."
            )
            return
        try:
            self.choices_list.clear()
        except Exception as e:
            logging.exception(f"ERROR during self.choices_list.clear(): {e}")
            return

        if node_data is None:
            logging.debug(
                "No node data provided to _update_properties_connections_list (likely no node selected)."
            )
            if self.add_choice_button:
                self.add_choice_button.setEnabled(False)
            if self.set_next_button:
                self.set_next_button.setEnabled(False)
            if self.clear_connections_button:
                self.clear_connections_button.setEnabled(False)
            if self.edit_choice_button:
                self.edit_choice_button.setEnabled(False)
            return

        logging.debug(
            f"Populating list for node '{node_data.id}'. Next: {node_data.next_node}, Choices: {node_data.choices}"
        )

        missing_link_color = getattr(
            config, "MISSING_LINK_TEXT_COLOR", QColor("orange")
        )
        implicit_loop_color = getattr(config, "NODE_LOOP_COLOR", QColor(100, 100, 0))
        next_link_color = getattr(config, "NEXT_LINK_TEXT_COLOR", QColor(180, 180, 255))
        default_choice_color = getattr(
            config, "DEFAULT_CHOICE_TEXT_COLOR", QColor(220, 220, 220)
        )

        has_explicit_connection = False
        if node_data.next_node is not None:
            target_id = node_data.next_node
            target_exists = target_id in self.nodes_data
            item_text = f"[NEXT] -> {target_id}"
            if not target_exists:
                item_text += " (Missing!)"
            item = QListWidgetItem(item_text)
            item.setForeground(next_link_color if target_exists else missing_link_color)
            item.setData(Qt.ItemDataRole.UserRole, (config.CONN_TYPE_NEXT, target_id))
            item.setFlags(
                item.flags()
                & ~Qt.ItemFlag.ItemIsSelectable
                & ~Qt.ItemFlag.ItemIsEditable
            )
            self.choices_list.addItem(item)
            has_explicit_connection = True
        elif node_data.choices:
            if isinstance(node_data.choices, list):
                for i, choice_data in enumerate(node_data.choices):
                    if isinstance(choice_data, (list, tuple)) and len(choice_data) == 3:
                        ct, tid, pre = choice_data
                        target_exists = tid in self.nodes_data
                        pre_str = f" [{pre}]" if pre and pre != "None" else ""
                        item_text = f'"{ct}"{pre_str} -> {tid}'
                        if not target_exists:
                            item_text += " (Missing!)"
                        item = QListWidgetItem(item_text)
                        item.setForeground(
                            default_choice_color
                            if target_exists
                            else missing_link_color
                        )
                        item.setData(
                            Qt.ItemDataRole.UserRole,
                            (config.CONN_TYPE_CHOICE, ct, tid, pre, i),
                        )  
                        self.choices_list.addItem(item)
                        has_explicit_connection = True
                    else:
                        logging.warning(
                            f"      Skipping invalid choice format at index {i} for node '{node_data.id}': {repr(choice_data)}"
                        )
            else:
                logging.warning(
                    f"    Choices attribute for node '{node_data.id}' is not a list: {type(node_data.choices)}"
                )

        is_end_node = not has_explicit_connection
        is_not_start_node = (
            self.start_node_id is not None and node_data.id != self.start_node_id
        )
        if is_end_node and is_not_start_node:
            target_id = self.start_node_id
            target_exists = target_id in self.nodes_data
            item_text = f"[IMPLICIT LOOP] -> {target_id}"
            if not target_exists:
                item_text += " (Missing!)"
            item = QListWidgetItem(item_text)
            item.setForeground(
                implicit_loop_color if target_exists else missing_link_color
            )
            item.setData(
                Qt.ItemDataRole.UserRole, (config.CONN_TYPE_IMPLICIT_LOOP, target_id)
            )
            item.setFlags(
                item.flags()
                & ~Qt.ItemFlag.ItemIsSelectable
                & ~Qt.ItemFlag.ItemIsEditable
            )
            self.choices_list.addItem(item)
            logging.debug(f"  Added IMPLICIT LOOP item: {item_text}")

        can_add_choice = node_data.next_node is None
        can_set_next = not node_data.choices
        if self.add_choice_button:
            self.add_choice_button.setEnabled(can_add_choice)
        if self.set_next_button:
            self.set_next_button.setEnabled(can_set_next)
        if self.clear_connections_button:
            self.clear_connections_button.setEnabled(has_explicit_connection)

        self.handle_current_choice_item_changed(self.choices_list.currentItem(), None)
        logging.debug(
            f" Finished updating connections list for '{node_data.id}'. Items: {self.choices_list.count()}"
        )

    def update_properties_panel(self):
        scene = getattr(self, "scene", None)
        props_widget = getattr(self, "props_widget", None)
        choices_list = getattr(self, "choices_list", None)
        try:
            if scene is None or props_widget is None or choices_list is None:
                logging.debug(
                    "update_properties_panel: Essential components (scene/props/choices) are None, skipping update."
                )
                return
            scene.sceneRect()  
            props_widget.isEnabled()
            choices_list.count()
        except RuntimeError as e:
            if "deleted" in str(e).lower():
                logging.debug(
                    f"update_properties_panel: Caught RuntimeError accessing deleted UI component: {e}. Skipping update."
                )
                return
            else:
                logging.exception(
                    "Unexpected RuntimeError in update_properties_panel validity check"
                )
                raise
        except Exception as e:
            logging.exception(
                f"Unexpected error during update_properties_panel validity check: {e}"
            )
            return

        logging.debug(
            f">>> update_properties_panel called (valid checks passed). self.choices_list is: {self.choices_list}"
        )
        selected_node = self.get_selected_node()

        scene = getattr(self, "scene", None)
        if scene is None:
            logging.debug(
                "update_properties_panel: Scene became None after get_selected_node, skipping."
            )
            return
        try:
            scene.sceneRect()  
        except RuntimeError as e:
            if "deleted" in str(e).lower():
                logging.debug(
                    f"update_properties_panel: Scene deleted after get_selected_node: {e}. Skipping."
                )
                return
            else:
                raise
        except Exception as e:
            logging.exception(f"Unexpected error re-checking scene validity: {e}")
            return

        is_node_selected = selected_node is not None

        if hasattr(self, "props_widget") and self.props_widget:
            self.props_widget.setEnabled(is_node_selected)
        if hasattr(self, "custom_props_group") and self.custom_props_group:
            self.custom_props_group.setEnabled(
                is_node_selected and bool(config.ALLOWED_CUSTOM_PROPERTIES)
            )
        if hasattr(self, "choices_list") and self.choices_list:
            self.choices_list.setEnabled(is_node_selected)
        if (
            hasattr(self, "edit_choice_button")
            and self.edit_choice_button
            and not is_node_selected
        ):
            self.edit_choice_button.setEnabled(
                False
            )  

        if not is_node_selected:

            if hasattr(self, "node_id_edit") and self.node_id_edit:
                self.node_id_edit.clear()
                self.node_id_edit.setReadOnly(True)
            if hasattr(self, "node_char_edit") and self.node_char_edit:
                self.node_char_edit.clear()
            if hasattr(self, "node_text_edit") and self.node_text_edit:
                self.node_text_edit.clear()
            self._update_properties_custom_fields(None)
            self._update_properties_connections_list(None)
            self._original_node_id = None
            self._original_node_char = None
            self._original_node_text = None
            self._original_custom_props.clear()
            return

        if hasattr(self, "node_id_edit") and self.node_id_edit:
            self.node_id_edit.blockSignals(True)
        if hasattr(self, "node_char_edit") and self.node_char_edit:
            self.node_char_edit.blockSignals(True)
        if hasattr(self, "node_text_edit") and self.node_text_edit:
            self.node_text_edit.blockSignals(True)
        if hasattr(self, "choices_list") and self.choices_list:
            self.choices_list.blockSignals(True)

        try:
            node_data = selected_node.node_data
            if not node_data:
                logging.error("Selected node has no node_data attribute!")
                return  

            self._update_properties_node_fields(node_data)
            self._update_properties_custom_fields(node_data)
            self._update_properties_connections_list(node_data)

        except Exception as e:
            logging.exception(
                f"Error updating properties panel for node '{getattr(selected_node.node_data, 'id', 'N/A')}': {e}"
            )
        finally:

            if hasattr(self, "node_id_edit") and self.node_id_edit:
                self.node_id_edit.blockSignals(False)
            if hasattr(self, "node_char_edit") and self.node_char_edit:
                self.node_char_edit.blockSignals(False)
            if hasattr(self, "node_text_edit") and self.node_text_edit:
                self.node_text_edit.blockSignals(False)
            if hasattr(self, "choices_list") and self.choices_list:
                self.choices_list.blockSignals(False)

    def handle_node_id_change(self):
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        selected_node = self.get_selected_node()
        if not selected_node or self._original_node_id is None:
            return

        old_id = self._original_node_id
        new_id_raw = self.node_id_edit.text()
        new_id = new_id_raw.strip()

        if not new_id:
            QMessageBox.warning(self, "Invalid ID", "Node ID cannot be empty.")
            self.node_id_edit.setText(old_id)
            return
        if " " in new_id:
            QMessageBox.warning(self, "Invalid ID", "Node ID cannot contain spaces.")
            self.node_id_edit.setText(old_id)
            return
        if (
            new_id == config.START_NODE_EXPORT_ID
            and old_id != config.START_NODE_EXPORT_ID
        ):
            QMessageBox.warning(
                self,
                "Invalid Rename",
                f"Cannot rename node to '{config.START_NODE_EXPORT_ID}' directly. Use the 'Set as Start Node' action.",
            )
            self.node_id_edit.setText(old_id)
            return
        if new_id == old_id:
            if new_id_raw != new_id:
                self.node_id_edit.setText(
                    new_id
                )  
            return

        if (
            new_id in self.nodes_data
            and self.nodes_data[new_id] != selected_node.node_data
        ):
            QMessageBox.warning(
                self, "ID Conflict", f"Node ID '{new_id}' already exists."
            )
            self.node_id_edit.setText(old_id)
            return

        cmd = RenameNodeIdCommand(old_id, new_id, self)
        self.undo_stack.push(cmd)

    def handle_defined_custom_prop_change(self):
        sender_widget = self.sender()
        if not isinstance(sender_widget, QLineEdit):
            return
        selected_node = self.get_selected_node()
        if not selected_node:
            return

        internal_key = sender_widget.property("internal_key")
        prop_config = config.ALLOWED_CUSTOM_PROPERTIES.get(internal_key)
        if not internal_key or not prop_config:
            logging.warning(
                f"Invalid internal key '{internal_key}' on custom prop widget change."
            )
            return

        new_value_str = sender_widget.text()
        old_value_str = self._original_custom_props.get(
            internal_key
        )  

        expected_type_str = prop_config.get("type", "string").lower()
        validated_new_value = new_value_str  
        is_valid = True
        try:
            if expected_type_str == "int":
                validated_new_value = int(new_value_str)
            elif expected_type_str == "float":
                validated_new_value = float(new_value_str)
            elif expected_type_str == "bool":
                val_lower = new_value_str.lower()
                if val_lower in ["true", "1", "yes", "on"]:
                    validated_new_value = True
                elif val_lower in ["false", "0", "no", "off"]:
                    validated_new_value = False
                else:
                    raise ValueError("Invalid boolean string")
        except (ValueError, TypeError):
            is_valid = False
            QMessageBox.warning(
                self,
                "Invalid Value",
                f"Value '{new_value_str}' is not a valid {expected_type_str} for property '{prop_config.get('display', internal_key)}'.",
            )
            sender_widget.setText(
                old_value_str if old_value_str is not None else ""
            )  

        if not is_valid:
            return

        if old_value_str is not None and new_value_str != old_value_str:
            logging.debug(
                f"Custom prop '{internal_key}' changed: '{old_value_str}' -> '{new_value_str}' (Validated: {validated_new_value})"
            )

            original_value_typed = selected_node.node_data.custom_data.get(internal_key)
            cmd = SetCustomPropertyCommand(
                selected_node.node_data.id,
                internal_key,
                original_value_typed,
                validated_new_value,
                self,
            )
            self.undo_stack.push(cmd)

        elif old_value_str is None:
            logging.warning(
                f"Original value for custom prop '{internal_key}' not tracked. Change not saved."
            )

    def _check_pending_custom_prop_changes(self):
        selected_node = self.get_selected_node()
        if not selected_node or not self._custom_prop_edits:
            return

        logging.debug("Checking for pending custom property changes...")
        needs_update = False
        for internal_key, widget in self._custom_prop_edits.items():
            current_text = widget.text()
            original_text = self._original_custom_props.get(internal_key)
            if original_text is not None and current_text != original_text:
                logging.info(
                    f"Pending change detected for custom prop '{internal_key}'. Triggering save."
                )
                widget.editingFinished.emit()  
                needs_update = True

    def _select_target_node_dialog(
        self, title: str, current_target: Optional[str] = None
    ) -> Optional[str]:
        selected_node = self.get_selected_node()
        if not selected_node:
            return None
        current_node_id = selected_node.node_data.id
        current_character = selected_node.node_data.character  

        existing_ids = sorted(list(self.nodes_data.keys()))
        create_new_option = "[Create New Node]"
        target_options = [create_new_option] + existing_ids

        current_index = 0
        if current_target and current_target in target_options:
            try:
                current_index = target_options.index(current_target)
            except ValueError:
                pass  

        target_id_str, ok = QInputDialog.getItem(
            self, title, "Select Target Node ID:", target_options, current_index, False
        )

        if not ok or not target_id_str:
            return None

        if target_id_str == create_new_option:
            logging.debug("User chose to create a new target node.")
            source_pos = selected_node.scenePos()

            new_node_pos = source_pos + config.DEFAULT_NODE_POS_OFFSET
            attempts = 0
            max_attempts = 20
            while attempts < max_attempts:
                too_close = False
                for other_gnode in self.graphics_nodes.values():
                    if QLineF(other_gnode.scenePos(), new_node_pos).length() < 1.0:
                        new_node_pos += QPointF(20, 20)  
                        too_close = True
                        break
                if not too_close:
                    break
                attempts += 1
            if attempts == max_attempts:
                logging.warning(
                    "Could not find non-overlapping position for new node after multiple attempts."
                )

            new_node_id = self._get_unique_node_id()

            self.add_node_cmd(
                node_id=new_node_id, pos=new_node_pos, character=current_character
            )

            if new_node_id in self.nodes_data:
                return new_node_id
            else:
                logging.error(
                    "Failed to add new node via command in _select_target_node_dialog."
                )
                QMessageBox.critical(self, "Error", "Failed to create the new node.")
                return None
        else:
            return target_id_str

    def add_choice_dialog(self):
        selected_node = self.get_selected_node()
        if not selected_node:
            return
        node_id = selected_node.node_data.id
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()

        if selected_node.node_data.next_node:
            QMessageBox.warning(
                self,
                "Add Choice Failed",
                f"Node '{node_id}' already uses a 'Next' connection.\nClear it before adding choices.",
            )
            return

        choice_text_raw, ok1 = QInputDialog.getText(
            self,
            "Add Choice",
            f"Choice Text (leave blank for [NEXT])",
        )
        if not ok1:
            return
        choice_text = choice_text_raw.strip()

        target_id = self._select_target_node_dialog(
            f"Target Node for Choice '{choice_text}'"
        )
        if not target_id:
            return

        preset_names = list(config.PRESETS.keys())
        preset_name, ok3 = QInputDialog.getItem(
            self, "Select Preset", "UI Preset:", preset_names, 0, False
        )
        if not ok3:
            preset_name = "None"  

        final_choice_text = choice_text if choice_text else config.DEFAULT_CHOICE_TEXT

        cmd = AddChoiceCommand(node_id, final_choice_text, target_id, preset_name, self)
        self.undo_stack.push(cmd)

    def set_next_node_dialog(self):
        selected_node = self.get_selected_node()
        if not selected_node:
            return
        node_id = selected_node.node_data.id
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()

        if selected_node.node_data.choices:
            QMessageBox.warning(
                self,
                "Set Next Failed",
                f"Node '{node_id}' already has choices defined.\nClear them before setting a 'Next' connection.",
            )
            return

        current_target = selected_node.node_data.next_node
        target_id = self._select_target_node_dialog(
            "Select 'Next' Target Node", current_target=current_target
        )
        if not target_id:
            return



        cmd = SetNextNodeCommand(node_id, target_id, self)
        self.undo_stack.push(cmd)

    def link_selected_nodes_cmd(self):
        selected = self.get_selected_nodes()
        if len(selected) != 2:
            QMessageBox.warning(
                self, "Link Nodes Error", "Please select exactly two nodes to link."
            )
            return
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()

        node1, node2 = selected
        if node1.pos().y() < node2.pos().y() - 10:
            source_node, dest_node = node1, node2
        elif node2.pos().y() < node1.pos().y() - 10:
            source_node, dest_node = node2, node1
        else:
            source_node, dest_node = (
                (node1, node2) if node1.pos().x() <= node2.pos().x() else (node2, node1)
            )

        source_id = source_node.node_data.id
        dest_id = dest_node.node_data.id

        if source_node.node_data.next_node:
            QMessageBox.warning(
                self,
                "Link Nodes Error",
                f"Source node '{source_id}' already uses a 'Next' connection.\nCannot add a choice link.",
            )
            return

        choice_text_raw, ok = QInputDialog.getText(
            self,
            "Link Nodes - Add Choice",
            f"Text for link '{source_id}' -> '{dest_id}':",
        )
        if not ok:
            return
        choice_text = choice_text_raw.strip()

        preset_names = list(config.PRESETS.keys())
        preset_name, ok_p = QInputDialog.getItem(
            self, "Select Preset", "UI Preset:", preset_names, 0, False
        )
        if not ok_p:
            preset_name = "None"

        final_choice_text = choice_text if choice_text else config.DEFAULT_CHOICE_TEXT

        cmd = AddChoiceCommand(source_id, final_choice_text, dest_id, preset_name, self)
        self.undo_stack.push(cmd)

    def show_choices_context_menu(self, pos: QPoint):
        selected_node_in_panel = self.get_selected_node()
        if not selected_node_in_panel or not self.choices_list:
            return
        item = self.choices_list.itemAt(pos)
        if not item:
            return
        item_data = item.data(Qt.ItemDataRole.UserRole)
        if not item_data or not isinstance(item_data, (list, tuple)):
            return

        connection_type = item_data[0]
        menu = QMenu()

        if connection_type == config.CONN_TYPE_CHOICE:
            remove_action = menu.addAction("Remove Choice")
            edit_action = menu.addAction("Edit Choice")
            menu.addSeparator()
            focus_action = menu.addAction("Focus Target Node")

            action = menu.exec(self.choices_list.mapToGlobal(pos))

            if action == remove_action:
                self._push_text_edit_command()
                self._push_char_edit_command()
                self._check_pending_custom_prop_changes()
                try:

                    choice_tuple = (item_data[1], item_data[2], item_data[3])
                    original_index = item_data[4]
                    if not isinstance(original_index, int) or original_index < 0:
                        raise ValueError("Invalid index stored")
                    cmd = RemoveChoiceCommand(
                        selected_node_in_panel.node_data.id,
                        original_index,
                        choice_tuple,
                        self,
                    )
                    self.undo_stack.push(cmd)
                except (IndexError, ValueError, TypeError) as e:
                    logging.exception(f"Error preparing remove choice command: {e}")
                    QMessageBox.warning(self, "Error", f"Could not remove choice: {e}")
            elif action == edit_action:
                self.edit_choice_item_cmd(item)
            elif action == focus_action:
                try:
                    self.handle_focus_request(item_data[2])
                except IndexError:
                    logging.warning(
                        "Could not get target ID from choice item for focus."
                    )

        elif (
            connection_type == config.CONN_TYPE_NEXT
            or connection_type == config.CONN_TYPE_IMPLICIT_LOOP
        ):
            focus_action = menu.addAction("Focus Target Node")
            clear_next_action = None
            if connection_type == config.CONN_TYPE_NEXT:
                clear_next_action = menu.addAction("Clear 'Next' Link")

            action = menu.exec(self.choices_list.mapToGlobal(pos))

            if action == focus_action:
                try:
                    self.handle_focus_request(item_data[1])
                except IndexError:
                    logging.warning("Could not get target ID from item for focus.")
            elif action == clear_next_action:
                self._push_text_edit_command()
                self._push_char_edit_command()
                self._check_pending_custom_prop_changes()
                cmd = SetNextNodeCommand(
                    selected_node_in_panel.node_data.id, None, self
                )
                self.undo_stack.push(cmd)

    def handle_focus_request(self, target_id: str):
        if target_id and target_id in self.graphics_nodes:
            target_gnode = self.graphics_nodes[target_id]
            logging.info(f"Focusing on node: {target_id}")
            if self.scene:
                self.scene.clearSelection()
            target_gnode.setSelected(True)
            if self.view:
                self.view.centerOn(target_gnode)
            target_gnode.flash()
        elif target_id:
            logging.warning(
                f"Focus target node '{target_id}' not found in graphics_nodes."
            )
            QMessageBox.warning(
                self, "Focus Error", f"Node '{target_id}' not found in the scene."
            )

    def handle_current_choice_item_changed(
        self,
        current_item: Optional[QListWidgetItem],
        previous_item: Optional[QListWidgetItem],
    ):
        can_edit = False
        if current_item:
            item_data = current_item.data(Qt.ItemDataRole.UserRole)
            if (
                item_data
                and isinstance(item_data, (list, tuple))
                and item_data[0] == config.CONN_TYPE_CHOICE
            ):
                can_edit = True
        if self.edit_choice_button:
            self.edit_choice_button.setEnabled(can_edit)

    def handle_edit_choice_button_clicked(self):
        if not self.choices_list:
            return
        current_item = self.choices_list.currentItem()
        if current_item:
            self.edit_choice_item_cmd(current_item)
        else:
            QMessageBox.information(
                self, "Edit Choice", "Please select a choice from the list to edit."
            )

    def edit_choice_item_cmd(self, item: QListWidgetItem):
        selected_node = self.get_selected_node()
        if not selected_node or not item:
            return

        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()

        item_data = item.data(Qt.ItemDataRole.UserRole)
        if (
            not item_data
            or not isinstance(item_data, (list, tuple))
            or len(item_data) != 5
            or item_data[0] != config.CONN_TYPE_CHOICE
        ):
            logging.warning(
                "Attempted to edit non-choice item or item with invalid data."
            )
            return

        try:

            original_text, original_target_id, original_preset, original_index = (
                item_data[1:5]
            )
            if not isinstance(original_index, int) or original_index < 0:
                raise ValueError(f"Invalid index '{original_index}'")
            number_of_choices = len(selected_node.node_data.choices)
        except (IndexError, ValueError, TypeError) as e:
            logging.exception(
                f"Edit Choice Error - Invalid item data format: {item_data}. Error: {e}"
            )
            QMessageBox.critical(
                self, "Edit Error", "Could not read choice data for editing."
            )
            return

        prompt_text = "New choice text:"
        if number_of_choices == 1:
            prompt_text += "\n(Leave blank to convert to 'Next')"
        else:
            prompt_text += "\n(Cannot leave blank)"

        new_text_raw, ok1 = QInputDialog.getText(
            self, "Edit Choice Text / Set Next", prompt_text, text=original_text
        )
        if not ok1:
            return
        new_text = new_text_raw.strip()

        if not new_text and number_of_choices == 1:
            logging.debug(
                f"Edit Choice: Blank text entered for the single choice. Converting to 'Next'."
            )
            new_target_id = self._select_target_node_dialog(
                f"Select Target Node for 'Next' (was choice '{original_text}')",
                current_target=original_target_id,
            )
            if not new_target_id:
                return
            if new_target_id == selected_node.node_data.id:
                QMessageBox.warning(
                    self, "Invalid Target", "Node cannot link to itself using 'Next'."
                )
                return

            cmd = SetNextNodeCommand(selected_node.node_data.id, new_target_id, self)
            self.undo_stack.push(cmd)
            logging.info(
                f"Pushed SetNextNodeCommand (from edited single choice) for {selected_node.node_data.id} -> {new_target_id}"
            )
            return  

        elif not new_text and number_of_choices > 1:
            QMessageBox.warning(
                self,
                "Edit Invalid",
                "Choice text cannot be left blank when multiple choices exist.\nDelete other choices first if you want to convert this to a 'Next' link.",
            )
            return  

        elif new_text:
            logging.debug(
                f"Edit Choice: Non-blank text '{new_text}' entered. Editing choice at index {original_index}."
            )
            new_target_id = self._select_target_node_dialog(
                "Select New Target Node", current_target=original_target_id
            )
            if not new_target_id:
                return

            preset_names = list(config.PRESETS.keys())
            try:
                current_preset_index = preset_names.index(original_preset)
            except ValueError:
                current_preset_index = 0  
            new_preset, ok3 = QInputDialog.getItem(
                self,
                "Select New Preset",
                "New UI Preset:",
                preset_names,
                current_preset_index,
                False,
            )
            if not ok3:
                return

            old_choice_tuple = (original_text, original_target_id, original_preset)
            new_choice_tuple = (new_text, new_target_id, new_preset)

            if old_choice_tuple != new_choice_tuple:
                cmd = EditChoiceCommand(
                    selected_node.node_data.id,
                    original_index,
                    old_choice_tuple,
                    new_choice_tuple,
                    self,
                )
                self.undo_stack.push(cmd)
                logging.info(
                    f"Pushed EditChoiceCommand for {selected_node.node_data.id}[{original_index}]"
                )
            else:
                logging.info("Edit Choice: No changes detected.")
            return  

        logging.error("Edit Choice: Reached end of logic unexpectedly.")

    def request_text_edit_focus(self, node_id: str):
        if node_id in self.graphics_nodes:
            gnode = self.graphics_nodes[node_id]
            if not gnode.isSelected():
                if self.scene:
                    self.scene.clearSelection()
                gnode.setSelected(True)
                QApplication.processEvents()  

            if self.node_text_edit and self.get_selected_node() == gnode:
                self.node_text_edit.setFocus()

                cursor = self.node_text_edit.textCursor()
                cursor.select(QTextCursor.SelectionType.Document)
                self.node_text_edit.setTextCursor(cursor)
                logging.debug(f"Focused text edit for node '{node_id}'.")
            else:
                logging.warning(
                    f"Could not focus text edit for node '{node_id}' - panel/selection mismatch?"
                )
        else:
            logging.warning(f"Node '{node_id}' requested for text focus not found.")

    def handle_next_to_choice_request(self, item: QListWidgetItem):
        selected_node = self.get_selected_node()
        if not selected_node or not item:
            return
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()

        item_data = item.data(Qt.ItemDataRole.UserRole)
        if (
            not item_data
            or not isinstance(item_data, (list, tuple))
            or len(item_data) < 2
            or item_data[0] != config.CONN_TYPE_NEXT
        ):
            logging.warning(
                "Attempted to convert non-NEXT item or item with invalid data."
            )
            return

        try:
            target_id = item_data[1]
            source_id = selected_node.node_data.id
        except IndexError:
            logging.exception(
                f"Convert NEXT Error - Invalid item data format: {item_data}."
            )
            QMessageBox.critical(
                self,
                "Conversion Error",
                "Could not read NEXT link data for conversion.",
            )
            return

        new_text = ""
        ok1 = False
        while not new_text:  
            new_text_raw, ok1 = QInputDialog.getText(
                self,
                "Convert to Choice",
                f"Enter Text for new choice linking to '{target_id}':\n(Cannot be blank)",
            )
            if not ok1:
                return  
            new_text = new_text_raw.strip()
            if not new_text:
                QMessageBox.warning(
                    self, "Text Required", "Choice text cannot be empty."
                )

        preset_names = list(config.PRESETS.keys())
        new_preset, ok2 = QInputDialog.getItem(
            self, "Select Preset", "UI Preset:", preset_names, 0, False
        )
        if not ok2:
            new_preset = "None"

        cmd = ConvertNextToChoiceCommand(
            source_id, target_id, new_text, new_preset, self
        )
        self.undo_stack.push(cmd)

    def get_edges_for_node(self, graphics_node: GraphicsNode) -> List[GraphicsEdge]:
        if not graphics_node or not hasattr(graphics_node, "node_data"):
            return []
        node_id = graphics_node.node_data.id
        connected_edges = []
        for edge in self.graphics_edges:
            source_valid = edge.source and hasattr(edge.source, "node_data")
            dest_valid = edge.dest and hasattr(edge.dest, "node_data")
            if source_valid and edge.source.node_data.id == node_id:
                connected_edges.append(edge)
            elif dest_valid and edge.dest.node_data.id == node_id:
                if edge not in connected_edges:
                    connected_edges.append(
                        edge
                    )  
        return connected_edges

    def draw_edges_for_node(self, graphics_node: GraphicsNode):
        if not graphics_node or not hasattr(graphics_node, "node_data"):
            logging.warning("draw_edges_for_node called with invalid node.")
            return
        if (
            not hasattr(self, "graphics_nodes")
            or not self.scene
            or not hasattr(self, "graphics_edges")
        ):
            logging.error("Editor components missing for drawing edges.")
            return

        node_id = graphics_node.node_data.id
        current_start_node_id = self.start_node_id
        logging.debug(
            f"Redrawing edges for node '{node_id}' (Start node is '{current_start_node_id}')..."
        )

        edges_to_remove = list(graphics_node.edges)  
        logging.debug(
            f" Found {len(edges_to_remove)} existing outgoing edges for '{node_id}'."
        )
        for edge in edges_to_remove:
            self.remove_edge_object(edge)

        node_data = graphics_node.node_data
        edges_created_count = 0

        if node_data.next_node:
            target_id = node_data.next_node
            if target_id == current_start_node_id:
                logging.debug(
                    f"  Skipping draw: 'Next' edge {node_id} -> {target_id} (loop to start)"
                )
            else:
                dest_node = self.graphics_nodes.get(target_id)
                if dest_node:
                    edge = GraphicsEdge(graphics_node, dest_node, self)
                    self.scene.addItem(edge)
                    self.graphics_edges.append(edge)
                    edges_created_count += 1
                    logging.debug(f"  Created 'Next' edge: {node_id} -> {target_id}")
                else:
                    logging.warning(
                        f"Target node '{target_id}' for next link from '{node_id}' not found. Edge not drawn."
                    )

        elif node_data.choices:
            for choice_text, target_id, preset_name in node_data.choices:
                if target_id == current_start_node_id:
                    logging.debug(
                        f"  Skipping draw: 'Choice' edge {node_id} -> {target_id} ('{choice_text}') (loop to start)"
                    )
                    continue
                dest_node = self.graphics_nodes.get(target_id)
                if dest_node:
                    edge = GraphicsEdge(
                        graphics_node, dest_node, self, choice_text=choice_text
                    )
                    self.scene.addItem(edge)
                    self.graphics_edges.append(edge)
                    edges_created_count += 1
                    logging.debug(
                        f"  Created 'Choice' edge: {node_id} -> {target_id} ('{choice_text}')"
                    )
                else:
                    logging.warning(
                        f"Target node '{target_id}' for choice '{choice_text}' from '{node_id}' not found. Edge not drawn."
                    )

        graphics_node.update()  
        logging.debug(
            f"Finished redrawing edges for '{node_id}'. Created {edges_created_count} non-looping edges."
        )

    def remove_edge_object(self, edge: GraphicsEdge):
        if not edge:
            return
        edge_desc = f"{getattr(edge.source.node_data, 'id', '?')}->{getattr(edge.dest.node_data, 'id', '?')}"

        if edge.source:
            edge.source.remove_edge(edge)

        if edge in self.graphics_edges:
            try:
                self.graphics_edges.remove(edge)
            except ValueError:
                logging.warning(f"Edge {edge_desc} already removed from master list.")
                pass
        if edge.scene() == self.scene:
            try:
                self.scene.removeItem(edge)
                logging.debug(f"Removed edge {edge_desc} from scene.")
            except Exception as e:
                logging.exception(
                    f"Error removing edge {edge_desc} item from scene: {e}"
                )
        elif edge.scene():
            logging.warning(f"Edge {edge_desc} found but belongs to a different scene.")

    def redraw_all_edges(self):
        logging.info("Redrawing all edges...")
        edges_to_clear = list(self.graphics_edges)
        logging.debug(f" Clearing {len(edges_to_clear)} existing graphical edges.")
        for edge in edges_to_clear:
            self.remove_edge_object(edge)

        if self.graphics_edges:
            logging.warning(
                f"graphics_edges list not empty after clear ({len(self.graphics_edges)} remaining). Forcibly clearing list."
            )
            self.graphics_edges.clear()

        nodes_processed = 0
        for node_id, graphics_node in self.graphics_nodes.items():
            self.draw_edges_for_node(graphics_node)
            nodes_processed += 1
        logging.info(f"Finished redrawing all edges for {nodes_processed} nodes.")

    def handle_edge_drag_started(self, source_node: GraphicsNode, start_pos: QPointF):
        if self.is_drawing_edge:
            logging.warning(
                "Attempted to start edge drag while another is in progress."
            )
            return
        if source_node.node_data.next_node:
            QMessageBox.warning(
                self,
                "Link Error",
                f"Node '{source_node.node_data.id}' already has a 'Next' connection.\nClear it first to draw choice links.",
            )
            return

        logging.debug(f"Starting edge drag from node '{source_node.node_data.id}'.")
        self.is_drawing_edge = True
        self.edge_drag_source_node = source_node
        self.edge_drag_line = QGraphicsLineItem()
        pen = QPen(
            config.EDGE_DRAG_COLOR, config.EDGE_PEN_WIDTH + 0.5, Qt.PenStyle.DashLine
        )
        self.edge_drag_line.setPen(pen)
        self.edge_drag_line.setZValue(2)  
        self.edge_drag_line.setLine(QLineF(start_pos, start_pos))
        if self.scene:
            self.scene.addItem(self.edge_drag_line)
        if self.view:
            self.view.setDragMode(
                QGraphicsView.DragMode.NoDrag
            )  
            self.view.setCursor(Qt.CursorShape.CrossCursor)

    def handle_edge_drag_moved(self, current_pos: QPointF):
        if not self.is_drawing_edge or not self.edge_drag_line:
            return
        line = self.edge_drag_line.line()
        line.setP2(current_pos)
        self.edge_drag_line.setLine(line)

    def handle_edge_drag_ended(self, scene_pos: QPointF):
        source_node = self.edge_drag_source_node
        self._cleanup_edge_drag()  
        if not source_node:
            logging.warning("Edge drag ended, but source node was lost.")
            return

        logging.debug(f"Edge drag ended at scene pos: {scene_pos}")
        item_at_release = (
            self.view.itemAt(self.view.mapFromScene(scene_pos)) if self.view else None
        )
        target_node: Optional[GraphicsNode] = None

        if isinstance(item_at_release, GraphicsNode) and item_at_release != source_node:
            target_node = item_at_release
            logging.info(f"Edge drag ended on target node: {target_node.node_data.id}")
        elif item_at_release:
            logging.debug(f"Edge drag ended on non-node item: {type(item_at_release)}")
        else:
            logging.debug("Edge drag ended on empty space.")

        if target_node:
            source_id = source_node.node_data.id
            target_id = target_node.node_data.id

            if source_node.node_data.next_node:
                logging.warning(
                    f"Drag link cancelled post-drag: source '{source_id}' has 'Next' connection."
                )
                return  

            choice_text_raw, ok = QInputDialog.getText(
                self,
                "Add Choice Link",
                f"Text for choice '{source_id}' -> '{target_id}':",
            )
            if not ok:
                logging.info("User cancelled adding choice text after edge drag.")
                return
            choice_text = choice_text_raw.strip()

            preset_names = list(config.PRESETS.keys())
            preset_name, ok_p = QInputDialog.getItem(
                self, "Select Preset", "UI Preset:", preset_names, 0, False
            )
            if not ok_p:
                preset_name = "None"

            final_choice_text = (
                choice_text if choice_text else config.DEFAULT_CHOICE_TEXT
            )

            cmd = AddChoiceCommand(
                source_id, final_choice_text, target_id, preset_name, self
            )
            self.undo_stack.push(cmd)

    def handle_edge_drag_cancelled(self):
        logging.debug("Edge drag cancelled.")
        self._cleanup_edge_drag()

    def _cleanup_edge_drag(self):
        if self.edge_drag_line and self.edge_drag_line.scene():
            self.scene.removeItem(self.edge_drag_line)
        self.edge_drag_line = None
        self.edge_drag_source_node = None
        self.is_drawing_edge = False
        if self.view:
            self.view.setDragMode(
                QGraphicsView.DragMode.ScrollHandDrag
            )  
            self.view.unsetCursor()
        logging.debug("Cleaned up edge drag state.")

    def get_selected_node(self) -> Optional[GraphicsNode]:
        scene = getattr(self, "scene", None)
        if scene is None:
            return None
        try:
            scene.sceneRect()  
        except RuntimeError:
            return None
        except Exception as e:
            logging.exception(
                f"Unexpected error accessing scene in get_selected_node: {e}"
            )
            return None

        try:
            items = scene.selectedItems()
            if len(items) == 1 and isinstance(items[0], GraphicsNode):
                return items[0]
        except RuntimeError:
            return None  
        except Exception as e:
            logging.exception(f"Unexpected error calling selectedItems: {e}")
            return None
        return None

    def get_selected_nodes(self) -> List[GraphicsNode]:
        if not self.scene:
            return []
        try:
            return [
                item
                for item in self.scene.selectedItems()
                if isinstance(item, GraphicsNode)
            ]
        except RuntimeError:
            return []

    def get_selected_edges(self) -> List[GraphicsEdge]:
        if not self.scene:
            return []
        try:
            return [
                item
                for item in self.scene.selectedItems()
                if isinstance(item, GraphicsEdge)
            ]
        except RuntimeError:
            return []

    def get_default_character_for_new_node(self) -> str:
        selected = self.get_selected_node()
        if selected and selected.node_data.character:
            return selected.node_data.character
        if self.start_node_id and self.start_node_id in self.nodes_data:
            start_char = self.nodes_data[self.start_node_id].character
            if start_char:
                return start_char
        return ""

    def show_about_dialog(self):
        version = getattr(config, "APP_VERSION", "Unknown")
        about_text = f"""<b>{config.APP_NAME}</b><br><br>Version: {version}<br>A visual tool for creating and editing node-based dialogues created by @wpenistone.<br><br>Uses PyQt6."""
        QMessageBox.about(self, f"About {config.APP_NAME} v{version}", about_text)

    def _run_auto_layout(self):
        logging.info("Run Auto Layout requested.")
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        self._apply_auto_layout()

    def _apply_auto_layout(self):
        NODE_HEIGHT = getattr(config, "NODE_HEIGHT", 100)
        NODE_WIDTH = getattr(config, "NODE_WIDTH", 200)
        LAYER_V_GAP = getattr(config, "LAYOUT_LAYER_V_GAP", NODE_HEIGHT + 80)
        NODE_H_GAP = getattr(config, "LAYOUT_NODE_H_GAP", 60)
        ORDERING_ITERATIONS = getattr(config, "LAYOUT_ORDERING_ITERATIONS", 16)
        DUMMY_NODE_WIDTH = getattr(config, "LAYOUT_DUMMY_WIDTH", 10)
        NUDGE_FACTOR = getattr(config, "LAYOUT_NUDGE_FACTOR", 0.3)
        NUDGE_ITERATIONS = getattr(config, "LAYOUT_NUDGE_ITERATIONS", 5)

        start_node_id_to_ignore = self.start_node_id
        if not self.nodes_data:
            print("Layout warning: No nodes to layout.")
            return
        if start_node_id_to_ignore and start_node_id_to_ignore not in self.nodes_data:
            print(
                f"Layout warning: Invalid start_node_id ('{start_node_id_to_ignore}'). Ignoring rule."
            )
            start_node_id_to_ignore = None

        print(
            f"Applying Sugiyama Layout (Ignoring edges to '{start_node_id_to_ignore}', BK + Nudge)..."
        )
        self._push_text_edit_command()  

        initial_canvas_offset_x, initial_canvas_offset_y = 50.0, 50.0

        real_adj_filtered = defaultdict(list)
        real_rev_adj_filtered = defaultdict(list)
        edges_to_process_for_dummy = []
        all_real_nodes = set(self.nodes_data.keys())
        for node_id, node_data in self.nodes_data.items():
            successors = []
            if (
                hasattr(node_data, "next_node")
                and node_data.next_node
                and node_data.next_node in self.nodes_data
            ):
                if node_data.next_node != start_node_id_to_ignore:
                    successors.append(node_data.next_node)
            elif hasattr(node_data, "choices") and node_data.choices:
                for _, tid, _ in node_data.choices:
                    if tid in self.nodes_data and tid != start_node_id_to_ignore:
                        successors.append(tid)

            unique_successors = sorted(list(set(successors)))  
            if unique_successors:
                real_adj_filtered[node_id].extend(unique_successors)
                for succ_id in unique_successors:
                    real_rev_adj_filtered[succ_id].append(node_id)
                    edges_to_process_for_dummy.append((node_id, succ_id))

        print("Phase 1: Layer Assignment...")
        node_levels = {}
        nodes_at_level = defaultdict(list)
        max_level = 0

        source_nodes = sorted(
            [nid for nid in all_real_nodes if not real_rev_adj_filtered.get(nid)]
        )
        if not source_nodes:  
            if self.start_node_id and self.start_node_id in all_real_nodes:
                source_nodes = [self.start_node_id]
            elif all_real_nodes:
                source_nodes = [sorted(list(all_real_nodes))[0]]
            else:
                print("Layout Error: No nodes found.")
                return

        queue = deque()
        visited_bfs_levels = {}
        for src_id in source_nodes:
            if src_id not in node_levels:
                node_levels[src_id] = 0
                nodes_at_level[0].append(src_id)
                queue.append((src_id, 0))
                visited_bfs_levels[src_id] = 0

        while queue:
            u, u_level = queue.popleft()
            max_level = max(max_level, u_level)
            for v in real_adj_filtered.get(u, []):
                new_v_level = u_level + 1
                current_v_level = visited_bfs_levels.get(v, -1)

                if new_v_level >= current_v_level:

                    if (
                        current_v_level != -1
                        and new_v_level > current_v_level
                        and v in nodes_at_level.get(current_v_level, [])
                    ):
                        try:
                            nodes_at_level[current_v_level].remove(v)
                        except ValueError:
                            pass  

                    node_levels[v] = new_v_level
                    if v not in nodes_at_level[new_v_level]:
                        nodes_at_level[new_v_level].append(v)
                    visited_bfs_levels[v] = new_v_level
                    queue.append(
                        (v, new_v_level)
                    )  

        processed_in_layering = set(node_levels.keys())
        for node_id in all_real_nodes:
            if node_id not in processed_in_layering:
                print(
                    f"Warning: Node '{node_id}' not reached by layering BFS. Assigning layer 0."
                )
                node_levels[node_id] = 0
                nodes_at_level[0].append(node_id)

        for level in nodes_at_level:
            nodes_at_level[level].sort()

        print("Phase 2: Adding Dummy Nodes...")
        adj = defaultdict(list)  
        rev_adj = defaultdict(list)  
        dummy_nodes = {}  
        all_node_ids_inc_dummies = set(self.nodes_data.keys())

        for u, successors in real_adj_filtered.items():
            u_level = node_levels.get(u, -1)
            if u_level == -1:
                continue  
            for v in successors:
                v_level = node_levels.get(v, -1)
                if v_level != -1 and v_level == u_level + 1:
                    adj[u].append(v)
                    rev_adj[v].append(u)

        dummy_idx_counter = 0
        for u, v in edges_to_process_for_dummy:
            u_level = node_levels.get(u)
            v_level = node_levels.get(v)
            if u_level is None or v_level is None:
                continue
            level_diff = v_level - u_level

            if level_diff > 1:
                last_node = u
                for i in range(level_diff - 1):
                    dummy_layer = u_level + i + 1

                    dummy_id = f"{config.DUMMY_NODE_PREFIX}{dummy_idx_counter}"
                    dummy_idx_counter += 1

                    dummy_nodes[dummy_id] = {
                        "layer": dummy_layer,
                        "edge": (u, v),
                        "pos": QPointF(),
                    }
                    node_levels[dummy_id] = dummy_layer
                    nodes_at_level[dummy_layer].append(dummy_id)
                    all_node_ids_inc_dummies.add(dummy_id)

                    adj[last_node].append(dummy_id)
                    rev_adj[dummy_id].append(last_node)
                    last_node = dummy_id

                adj[last_node].append(v)
                rev_adj[v].append(last_node)

            elif level_diff <= 0:
                print(
                    f"Layout Warning: Edge ({u} L{u_level}, {v} L{v_level}) ignored for dummy creation (cycle or layering issue)."
                )

        for level in nodes_at_level:
            nodes_at_level[level].sort()
        current_max_level = max(nodes_at_level.keys()) if nodes_at_level else -1

        print(
            f"Phase 3: Crossing Reduction ({ORDERING_ITERATIONS} iterations, Alt Bary/Median + Swap)..."
        )
        node_order_indices = {}  

        def update_node_order_indices():
            node_order_indices.clear()
            for level_idx_update in range(current_max_level + 1):
                ordered_nodes = nodes_at_level.get(level_idx_update, [])
                for index, node_id_update in enumerate(ordered_nodes):
                    node_order_indices[node_id_update] = index

        update_node_order_indices()

        total_swaps = 0
        for iteration in range(ORDERING_ITERATIONS):
            swaps_in_iter = 0
            use_median = iteration % 4 >= 2  

            pass_metrics = {}
            for level_idx in range(1, current_max_level + 1):
                for node_id in nodes_at_level.get(level_idx, []):
                    preds = [
                        p
                        for p in rev_adj.get(node_id, [])
                        if node_order_indices.get(p) is not None
                        and p in nodes_at_level.get(level_idx - 1, [])
                    ]
                    metric = node_order_indices.get(
                        node_id, 0.0
                    )  
                    if preds:
                        pred_indices = sorted(
                            [node_order_indices[pid] for pid in preds]
                        )
                        if pred_indices:
                            if use_median:
                                metric = pred_indices[(len(pred_indices) - 1) // 2]
                            else:
                                metric = sum(pred_indices) / len(pred_indices)
                    pass_metrics[node_id] = metric

            for level_idx in range(1, current_max_level + 1):
                nodes_at_level[level_idx].sort(
                    key=lambda nid: (
                        pass_metrics.get(nid, node_order_indices.get(nid, 0.0)),
                        nid,
                    )
                )  
            update_node_order_indices()

            down_swaps = 0
            for level_idx in range(1, current_max_level + 1):
                improved = True
                while improved:
                    improved = False
                    current_order = nodes_at_level[level_idx]
                    for i in range(len(current_order) - 1):
                        u, v = current_order[i], current_order[i + 1]
                        crossings_uv = count_crossings_between_nodes(
                            u,
                            v,
                            nodes_at_level,
                            node_order_indices,
                            rev_adj,
                            level_idx - 1,
                        )
                        crossings_vu = count_crossings_between_nodes(
                            v,
                            u,
                            nodes_at_level,
                            node_order_indices,
                            rev_adj,
                            level_idx - 1,
                        )
                        if crossings_vu < crossings_uv:
                            (
                                nodes_at_level[level_idx][i],
                                nodes_at_level[level_idx][i + 1],
                            ) = (v, u)
                            improved = True
                            swaps_in_iter += 1
                            down_swaps += 1
                            update_node_order_indices()  

            pass_metrics.clear()
            for level_idx in range(
                current_max_level - 1, -1, -1
            ):  
                for node_id in nodes_at_level.get(level_idx, []):
                    succs = [
                        s
                        for s in adj.get(node_id, [])
                        if node_order_indices.get(s) is not None
                        and s in nodes_at_level.get(level_idx + 1, [])
                    ]
                    metric = node_order_indices.get(node_id, 0.0)  
                    if succs:
                        succ_indices = sorted(
                            [node_order_indices[sid] for sid in succs]
                        )
                        if succ_indices:
                            if use_median:
                                metric = succ_indices[(len(succ_indices) - 1) // 2]
                            else:
                                metric = sum(succ_indices) / len(succ_indices)
                    pass_metrics[node_id] = metric

            for level_idx in range(current_max_level - 1, -1, -1):
                nodes_at_level[level_idx].sort(
                    key=lambda nid: (
                        pass_metrics.get(nid, node_order_indices.get(nid, 0.0)),
                        nid,
                    )
                )
            update_node_order_indices()

            up_swaps = 0
            for level_idx in range(current_max_level - 1, -1, -1):
                improved = True
                while improved:
                    improved = False
                    current_order = nodes_at_level[level_idx]
                    for i in range(len(current_order) - 1):
                        u, v = current_order[i], current_order[i + 1]
                        crossings_uv = count_crossings_between_nodes(
                            u, v, nodes_at_level, node_order_indices, adj, level_idx + 1
                        )
                        crossings_vu = count_crossings_between_nodes(
                            v, u, nodes_at_level, node_order_indices, adj, level_idx + 1
                        )
                        if crossings_vu < crossings_uv:
                            (
                                nodes_at_level[level_idx][i],
                                nodes_at_level[level_idx][i + 1],
                            ) = (v, u)
                            improved = True
                            swaps_in_iter += 1
                            up_swaps += 1
                            update_node_order_indices()

            total_swaps += swaps_in_iter
            if (
                swaps_in_iter == 0 and iteration > 4
            ):  
                print(f"  Converged after iteration {iteration + 1}.")
                break

        print(f"Phase 3: Crossing Reduction Complete. Total swaps: {total_swaps}")

        print("Phase 4: Coordinate Assignment (BK Down-Align + Nudge)...")
        root = {nid: nid for nid in all_node_ids_inc_dummies}
        align = {nid: nid for nid in all_node_ids_inc_dummies}
        x_coords = {nid: 0.0 for nid in all_node_ids_inc_dummies}  
        final_positions = {}  

        def get_node_width(nid):
            return NODE_WIDTH if nid in self.nodes_data else DUMMY_NODE_WIDTH

        def get_node_separation(n1_id, n2_id):
            w1 = get_node_width(n1_id)
            w2 = get_node_width(n2_id)
            return (w1 / 2.0) + (w2 / 2.0) + NODE_H_GAP

        for l in range(1, current_max_level + 1):  
            for v in nodes_at_level.get(l, []):
                preds = [
                    p for p in rev_adj.get(v, []) if p in nodes_at_level.get(l - 1, [])
                ]
                if preds:
                    preds.sort(key=lambda p: node_order_indices.get(p, float("inf")))
                    median_pred_idx = (len(preds) - 1) // 2
                    median_pred_id = preds[median_pred_idx]

                    if (
                        align.get(median_pred_id, median_pred_id) != median_pred_id
                        or v in dummy_nodes
                    ):
                        align[v] = median_pred_id

                        curr_root = root.get(median_pred_id, median_pred_id)
                        while root.get(curr_root, curr_root) != curr_root:
                            curr_root = root.get(curr_root, curr_root)
                        root[v] = curr_root

        placed_x = {}  
        for l in range(current_max_level + 1):
            last_placed_node_id = None
            for v in nodes_at_level.get(l, []):
                if v not in placed_x:  

                    curr_root = root.get(v, v)
                    while root.get(curr_root, curr_root) != curr_root:
                        curr_root = root.get(curr_root, curr_root)

                    min_pos = 0.0
                    if last_placed_node_id is not None:
                        separation = get_node_separation(last_placed_node_id, curr_root)
                        min_pos = placed_x.get(last_placed_node_id, 0.0) + separation

                    block_nodes = [
                        n
                        for n in nodes_at_level.get(l, [])
                        if root.get(n, n) == curr_root
                    ]
                    current_pos = min_pos
                    for node_in_block in block_nodes:
                        placed_x[node_in_block] = current_pos
                        last_placed_node_id = (
                            node_in_block  
                        )
                        current_pos += get_node_separation(
                            node_in_block, node_in_block
                        )  

        x_coords = placed_x.copy()  

        print(
            f"  BK Pass 4.5: Nudging nodes towards successors ({NUDGE_ITERATIONS} iters, factor={NUDGE_FACTOR})..."
        )
        for nudge_iter in range(NUDGE_ITERATIONS):
            total_nudge_this_iter = 0.0
            for l in range(current_max_level + 1):
                current_layer_nodes = nodes_at_level.get(l, [])
                num_nodes_in_layer = len(current_layer_nodes)
                for i, u in enumerate(current_layer_nodes):
                    successors = [
                        s
                        for s in adj.get(u, [])
                        if node_levels.get(s) == l + 1 and s in x_coords
                    ]
                    if successors:
                        try:
                            avg_succ_x = sum(x_coords[s] for s in successors) / len(
                                successors
                            )
                        except ZeroDivisionError:
                            continue

                        current_x = x_coords.get(u, 0.0)
                        delta = avg_succ_x - current_x
                        nudge = NUDGE_FACTOR * delta

                        min_allowed_x_left = -float("inf")
                        max_allowed_x_right = float("inf")
                        if i > 0:
                            left_neighbor = current_layer_nodes[i - 1]
                            left_neighbor_x = x_coords.get(left_neighbor, -float("inf"))
                            if left_neighbor_x > -float("inf"):
                                separation = get_node_separation(left_neighbor, u)
                                min_allowed_x_left = left_neighbor_x + separation
                        if i < num_nodes_in_layer - 1:
                            right_neighbor = current_layer_nodes[i + 1]
                            right_neighbor_x = x_coords.get(
                                right_neighbor, float("inf")
                            )
                            if right_neighbor_x < float("inf"):
                                separation = get_node_separation(u, right_neighbor)
                                max_allowed_x_right = right_neighbor_x - separation

                        target_x = current_x + nudge
                        final_x = max(
                            min_allowed_x_left, min(target_x, max_allowed_x_right)
                        )
                        actual_nudge = final_x - current_x
                        if abs(actual_nudge) > 1e-3:
                            x_coords[u] = final_x
                            total_nudge_this_iter += abs(actual_nudge)

            if total_nudge_this_iter < 1.0 and nudge_iter > 0:  
                print(f"    Nudging converged after iter {nudge_iter + 1}.")
                break

        print("  BK Pass 5: Global Centering and Finalizing Positions...")
        min_x_coord = float("inf")
        max_x_coord = -float("inf")
        nodes_have_coords = False
        for node_id in all_node_ids_inc_dummies:
            if node_id in x_coords:
                nodes_have_coords = True
                width = get_node_width(node_id)
                center_x = x_coords[node_id]
                min_x_coord = min(min_x_coord, center_x - width / 2.0)
                max_x_coord = max(max_x_coord, center_x + width / 2.0)

        x_offset = initial_canvas_offset_x
        if nodes_have_coords and max_x_coord > min_x_coord:
            layout_width = max_x_coord - min_x_coord
            current_center_x = min_x_coord + layout_width / 2.0
            x_offset = initial_canvas_offset_x - current_center_x
            print(
                f"  Layout bounds: [{min_x_coord:.1f}, {max_x_coord:.1f}]. Offset: {x_offset:.1f}"
            )
        elif nodes_have_coords:
            x_offset = initial_canvas_offset_x - min_x_coord  
            print(f"  Single column layout. Offset: {x_offset:.1f}")
        else:
            print("  Warning: No nodes have coords.")

        for l in range(current_max_level + 1):
            level_y_center = initial_canvas_offset_y + l * LAYER_V_GAP
            level_y_top = level_y_center - NODE_HEIGHT / 2.0  
            for v in nodes_at_level.get(l, []):
                if v in x_coords:
                    node_center_x = x_coords[v] + x_offset
                    if v in self.nodes_data:
                        node_width = get_node_width(v)
                        node_top_left_x = node_center_x - node_width / 2.0
                        final_positions[v] = QPointF(node_top_left_x, level_y_top)
                    elif v in dummy_nodes:
                        dummy_nodes[v]["pos"] = QPointF(
                            node_center_x, level_y_center
                        )  
                else:
                    print(f"  Warning: Node '{v}' in layer {l} has no final x-coord.")

        print("Phase 5: Checking for unplaced nodes...")
        placed_real_nodes = set(final_positions.keys())
        all_real_node_ids = set(self.nodes_data.keys())
        unplaced_nodes = sorted(list(all_real_node_ids - placed_real_nodes))
        if unplaced_nodes:
            print(
                f"Layout CRITICAL WARNING: {len(unplaced_nodes)} nodes missed by BK: {unplaced_nodes}. Placing arbitrarily."
            )
            max_y_placed = max(
                (p.y() + NODE_HEIGHT for p in final_positions.values()),
                default=initial_canvas_offset_y,
            )
            unplaced_start_y = max(initial_canvas_offset_y, max_y_placed + LAYER_V_GAP)
            current_unplaced_x = initial_canvas_offset_x
            for i, node_id in enumerate(unplaced_nodes):
                pos = QPointF(
                    current_unplaced_x + i * (NODE_WIDTH + NODE_H_GAP + 50),
                    unplaced_start_y,
                )
                final_positions[node_id] = pos

        print("Phase 6: Applying Positions...")
        old_positions = {}
        nodes_in_layout = set(final_positions.keys())
        for node_id, gnode in self.graphics_nodes.items():
            if node_id in nodes_in_layout:
                old_positions[node_id] = gnode.pos()

        if final_positions:
            layout_command_text = "Apply Sugiyama Layout (BK + Nudge)"
            needs_move = False
            for node_id, target_pos in final_positions.items():
                current_pos = old_positions.get(node_id)
                if current_pos is None:
                    needs_move = True
                    break
                if (
                    abs(current_pos.x() - target_pos.x()) > 0.1
                    or abs(current_pos.y() - target_pos.y()) > 0.1
                ):
                    needs_move = True
                    break

            if needs_move:
                print(f"Pushing '{layout_command_text}' command...")
                cmd = ApplyLayoutCommand(
                    old_positions, final_positions, self, text=layout_command_text
                )
                if hasattr(self, "scene"):
                    self.scene.clearSelection()
                if hasattr(QApplication, "processEvents"):
                    QApplication.processEvents()
                self.undo_stack.push(cmd)
                print("Layout command pushed and executed.")
            else:
                print(
                    "Layout complete, no significant node movements detected. No command pushed."
                )
                self.redraw_all_edges()
                self._fit_view_after_layout()
        else:
            print("Layout complete, no positions generated/applied.")

    def _fit_view_after_layout(self):
        if not self.scene or not self.view:
            logging.warning("Cannot fit view: Scene or View not available.")
            return
        try:

            all_items = self.scene.items()
            if not all_items:
                bounds = QRectF()
            else:
                bounds = all_items[0].sceneBoundingRect()
                for item in all_items[1:]:
                    bounds = bounds.united(item.sceneBoundingRect())

            if bounds.isValid() and not bounds.isEmpty():
                pad_x = getattr(config, "VIEW_FIT_PADDING_X", 100.0)
                pad_y = getattr(config, "VIEW_FIT_PADDING_Y", 100.0)
                padded_bounds = bounds.adjusted(-pad_x, -pad_y, pad_x, pad_y)
                self.view.setSceneRect(padded_bounds)
                self.view.fitInView(padded_bounds, Qt.AspectRatioMode.KeepAspectRatio)

                content_center = bounds.center()
                self.view.centerOn(content_center)
                logging.info(
                    f"Fit view. Bounds: {bounds}, Padded SceneRect: {padded_bounds}, Centered View On: {content_center}"
                )
            else:
                logging.info("Scene bounds invalid or empty, resetting view.")
                default_rect = QRectF(0, 0, 600, 400)
                self.view.setSceneRect(default_rect)
                self.view.fitInView(default_rect, Qt.AspectRatioMode.KeepAspectRatio)
        except Exception as e:
            logging.exception(f"Error fitting view: {e}")
            self.view.setSceneRect(QRectF(0, 0, 500, 500))  

    def new_file(self) -> bool:
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        if self.unsaved_changes:
            reply = QMessageBox.question(
                self,
                "Unsaved Changes",
                "Discard current unsaved changes?",
                QMessageBox.StandardButton.Discard | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if reply == QMessageBox.StandardButton.Cancel:
                logging.info("New file action cancelled by user.")
                return False

        logging.info("Creating new project...")
        if self.scene:
            self.scene.clearSelection()
            items_to_remove = list(self.scene.items())
            for item in items_to_remove:
                if isinstance(item, GraphicsEdge):
                    self.remove_edge_object(item)
                elif item in self.scene.items():  
                    try:
                        self.scene.removeItem(item)
                    except Exception as e:
                        logging.exception(
                            f"Error removing item {item} during clear: {e}"
                        )
            self.scene.clear()  

        self.nodes_data.clear()
        self.graphics_nodes.clear()
        self.graphics_edges.clear()
        self.visual_groups.clear()  
        self.bookmarks.clear()  
        self.clipboard.clear()  
        self.start_node_id = None
        self.next_node_id_counter = 1
        self.current_project_path = None
        self.undo_stack.clear()
        self._mark_unsaved(False)
        self.update_properties_panel()
        self._update_window_title()
        self.update_bookmark_menu()  
        self.clear_all_highlights()  
        logging.info("New project created successfully.")
        return True

    def load_project(self):
        if not self.new_file():
            return  

        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open Dialogue Project", "", config.PROJECT_FILE_FILTER
        )
        if not file_path:
            return

        try:
            logging.info(f"Loading project from: {file_path}")
            with open(file_path, "r", encoding="utf-8") as f:
                project_data = json.load(f)

            nodes_key = config.get_project_key("nodes")
            if not isinstance(project_data, dict) or nodes_key not in project_data:
                raise ValueError(
                    f"Invalid format: Missing top-level '{nodes_key}' dictionary."
                )
            loaded_nodes_dict = project_data.get(nodes_key, {})
            if not isinstance(loaded_nodes_dict, dict):
                raise ValueError(
                    f"Invalid format: Nodes data ('{nodes_key}') is not a dictionary."
                )

            potential_start_id: Optional[str] = None
            load_errors: List[str] = []
            nodes_added_count = 0
            for node_key, node_dict_data in loaded_nodes_dict.items():
                try:
                    node_data = DialogueNodeData.from_dict(node_dict_data)
                    if not node_data.id:
                        raise ValueError("Node data missing ID.")
                    if node_data.id in self.nodes_data:
                        raise ValueError(f"Duplicate node ID '{node_data.id}'")
                    graphics_node = self._add_node_internal(node_data)
                    if not graphics_node:
                        raise ValueError(f"Failed internal add for '{node_data.id}'")
                    nodes_added_count += 1
                    if node_data.is_start_node:
                        if potential_start_id is None:
                            potential_start_id = node_data.id
                        else:
                            logging.warning(
                                f"Multiple start nodes flagged ('{potential_start_id}', '{node_data.id}'). Using first found: '{potential_start_id}'."
                            )
                            node_data.is_start_node = (
                                False  
                            )
                except Exception as node_e:
                    logging.exception(f"Error loading node '{node_key}': {node_e}")
                    load_errors.append(f"Node '{node_key}': {node_e}")

            if load_errors:
                QMessageBox.warning(
                    self,
                    "Load Warning",
                    f"Errors occurred while loading some nodes:\n- "
                    + "\n- ".join(load_errors),
                )
            if nodes_added_count == 0 and not load_errors:
                raise ValueError(
                    "No valid nodes were loaded from the file."
                )  

            bookmarks_key = config.get_project_key("bookmarks")
            loaded_bookmarks = project_data.get(bookmarks_key, [])
            if isinstance(loaded_bookmarks, list):
                self.bookmarks = {
                    str(b_id)
                    for b_id in loaded_bookmarks
                    if isinstance(b_id, str) and b_id in self.nodes_data
                }
                logging.info(f"Loaded {len(self.bookmarks)} valid bookmarks.")
                self.update_bookmark_menu()
                self.update_all_dynamic_node_visuals()  
            else:
                logging.warning(
                    f"Invalid bookmark format found in project file (expected list)."
                )

            start_set = False
            if potential_start_id and potential_start_id in self.graphics_nodes:
                if self._set_editor_start_node(potential_start_id):
                    start_set = True
            elif not start_set and config.START_NODE_EXPORT_ID in self.graphics_nodes:
                if self._set_editor_start_node(config.START_NODE_EXPORT_ID):
                    start_set = True
            elif (
                not start_set and self.nodes_data
            ):  
                fallback_id = sorted(self.nodes_data.keys())[0]
                if fallback_id in self.graphics_nodes:
                    if self._set_editor_start_node(fallback_id):
                        start_set = True

            if (
                not start_set and nodes_added_count > 0
            ):  
                QMessageBox.critical(
                    self,
                    "Load Error",
                    "Failed to determine or set a start node for the loaded project.",
                )
                self.new_file()
                return

            self.redraw_all_edges()
            self.current_project_path = file_path
            self.undo_stack.clear()
            self.undo_stack.setClean()
            self._mark_unsaved(False)
            self.update_properties_panel()
            self._update_window_title()
            self._fit_view_after_layout()
            logging.info(f"Project loaded successfully: {len(self.nodes_data)} nodes.")

        except Exception as e:
            logging.exception("Failed to load project from {file_path}: {e}")
            QMessageBox.critical(self, "Load Failed", f"Could not load project:\n{e}")
            self.new_file()  

    def save_project_as(self) -> bool:
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        default_name_base = self.start_node_id if self.start_node_id else "Untitled"
        default_name = f"{default_name_base}{config.PROJECT_FILE_EXTENSION}"
        start_dir = (
            os.path.dirname(self.current_project_path)
            if self.current_project_path
            else ""
        )
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Dialogue Project As",
            os.path.join(start_dir, default_name),
            config.PROJECT_FILE_FILTER,
        )

        if not file_path:
            logging.info("Save As cancelled by user.")
            return False

        if not file_path.lower().endswith(config.PROJECT_FILE_EXTENSION.lower()):
            file_path += config.PROJECT_FILE_EXTENSION

        self.current_project_path = file_path
        self._update_window_title()  
        return self.save_project()

    def save_project(self) -> bool:
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        if not self.current_project_path:
            return self.save_project_as()

        nodes_key = config.get_project_key("nodes")
        bookmarks_key = config.get_project_key("bookmarks")
        groups_key = config.get_project_key("visual_groups")
        project_data = {nodes_key: {}, bookmarks_key: [], groups_key: []}

        try:
            logging.info(f"Saving project to: {self.current_project_path}")

            for node_id, node_data in self.nodes_data.items():
                if node_id in self.graphics_nodes:
                    node_data.pos = self.graphics_nodes[
                        node_id
                    ].pos()  
                else:
                    logging.warning(
                        f"Node '{node_id}' exists in data but not graphics during save. Position might be stale."
                    )
                project_data[nodes_key][node_id] = node_data.to_dict()

            project_data[bookmarks_key] = sorted(list(self.bookmarks))

            with open(self.current_project_path, "w", encoding="utf-8") as f:
                json.dump(project_data, f, indent=config.JSON_INDENT)

            self.undo_stack.setClean()
            self._mark_unsaved(False)
            logging.info("Project saved successfully.")
            return True
        except Exception as e:
            logging.exception(
                f"Could not save project to {self.current_project_path}: {e}"
            )
            QMessageBox.critical(self, "Save Failed", f"Could not save project:\n{e}")
            return False

    def import_from_json(self):
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        if self.unsaved_changes:
            reply = QMessageBox.question(
                self,
                "Unsaved Changes",
                "Importing will discard current unsaved changes. Continue?",
                QMessageBox.StandardButton.Discard | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if reply == QMessageBox.StandardButton.Cancel:
                return

        start_dir = (
            os.path.dirname(self.current_project_path)
            if self.current_project_path
            else ""
        )
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Import JSON File", start_dir, config.GAME_JSON_FILTER
        )
        if not file_path:
            return

        try:
            logging.info(f"Importing JSON from: {file_path}")
            with open(file_path, "r", encoding="utf-8") as f:
                loaded_data = json.load(f)
        except Exception as e:
            logging.exception(f"Error reading or parsing JSON file: {e}")
            QMessageBox.critical(
                self, "Import Failed", f"Error reading or parsing JSON file:\n{e}"
            )
            return

        if self.scene:
            self.scene.clearSelection()
            self.scene.clear()
        self.nodes_data.clear()
        self.graphics_nodes.clear()
        self.graphics_edges.clear()
        self.visual_groups.clear()
        self.bookmarks.clear()
        self.clipboard.clear()
        self.start_node_id = None
        self.next_node_id_counter = 1
        self.current_project_path = None
        self.undo_stack.clear()
        self._mark_unsaved(True)  
        self._update_window_title()
        self.update_bookmark_menu()

        if not isinstance(loaded_data, dict):
            QMessageBox.critical(
                self,
                "Import Error",
                "JSON root structure must be an object (dictionary). Import aborted.",
            )
            return

        try:
            temp_start_id = self._determine_start_node_id(loaded_data)
            if not temp_start_id:
                QMessageBox.critical(
                    self,
                    "Import Error",
                    "Cannot determine the start node from the JSON structure. Import aborted.",
                )
                self.new_file()
                return  

            id_map = self._create_nodes_from_import_data(loaded_data, temp_start_id)
            if not self.nodes_data:
                raise ValueError("No nodes were created from the import data.")

            if not self._set_editor_start_node(temp_start_id):
                self.new_file()
                return  

            self._populate_connections_from_import_data(loaded_data, id_map)
            self._run_auto_layout()  

            logging.info(f"Import complete: {len(self.nodes_data)} nodes.")
            QMessageBox.information(
                self,
                "Import Complete",
                f"Successfully imported dialogue from:\n{file_path}",
            )

            self.undo_stack.clear()  
            self.undo_stack.setClean()
            self._mark_unsaved(True)  
            self.update_properties_panel()  

        except Exception as e:
            logging.exception(f"An error occurred during the import process: {e}")
            QMessageBox.critical(
                self,
                "Import Error",
                f"An error occurred during the import process:\n{e}",
            )
            self.new_file()  

    def _determine_start_node_id(self, loaded_data: dict) -> Optional[str]:
        start_wrapper_key = config.get_game_key("start_node_wrapper")
        node_id_key = config.get_game_key("node_id")
        temp_start_node_id = None
        logging.debug(
            f"Determining start node: Wrapper key='{start_wrapper_key}', ID key='{node_id_key}'"
        )

        if start_wrapper_key and start_wrapper_key in loaded_data:
            start_node_content = loaded_data[start_wrapper_key]
            if isinstance(start_node_content, dict):
                temp_start_node_id = start_node_content.get(node_id_key)
                if temp_start_node_id:
                    logging.info(
                        f"Found start node ID '{temp_start_node_id}' within wrapper '{start_wrapper_key}'."
                    )
                    return str(temp_start_node_id)
                else:  
                    logging.warning(
                        f"Start wrapper '{start_wrapper_key}' exists but missing '{node_id_key}'. Assuming wrapper key IS the start node ID."
                    )
                    temp_start_node_id = start_wrapper_key

                    if temp_start_node_id in loaded_data and isinstance(
                        loaded_data[temp_start_node_id], dict
                    ):
                        return str(temp_start_node_id)
                    else:
                        logging.error(
                            f"Start node ID '{temp_start_node_id}' (from wrapper key) does not correspond to a node object in the JSON root."
                        )
                        return None
            elif isinstance(
                start_node_content, (str, int, float)
            ):  
                temp_start_node_id = str(start_node_content)

                if temp_start_node_id in loaded_data and isinstance(
                    loaded_data[temp_start_node_id], dict
                ):
                    logging.info(
                        f"Found start node ID '{temp_start_node_id}' as value of wrapper key '{start_wrapper_key}'."
                    )
                    return temp_start_node_id
                else:
                    logging.error(
                        f"Start node ID '{temp_start_node_id}' (from wrapper value) does not correspond to a node object in the JSON root."
                    )
                    return None

        if start_wrapper_key:
            for key, content in loaded_data.items():
                if key == start_wrapper_key:
                    continue  
                if isinstance(content, dict):
                    node_id_in_content = content.get(node_id_key)

                    if (
                        node_id_in_content
                        and str(node_id_in_content) == start_wrapper_key
                    ):
                        logging.info(
                            f"Found node '{key}' whose ID field ('{node_id_in_content}') matches start wrapper key '{start_wrapper_key}'. Using it as start."
                        )
                        return str(start_wrapper_key)

        if loaded_data:
            first_key = next(iter(loaded_data))
            if (
                first_key == start_wrapper_key
            ):  
                if len(loaded_data) > 1:
                    first_key = list(loaded_data.keys())[1]
                else:
                    logging.error(
                        "Only found start wrapper key in JSON, cannot determine fallback start node."
                    )
                    return None

            first_content = loaded_data.get(first_key)
            if isinstance(first_content, dict):

                temp_start_node_id = first_content.get(node_id_key, first_key)
                logging.warning(
                    f"No explicit start node found via wrapper key '{start_wrapper_key}'. Using first node key/ID '{temp_start_node_id}' as start."
                )
                QMessageBox.warning(
                    self,
                    "Import Warning",
                    f"Could not determine start node from configuration.\nUsing first node encountered ('{temp_start_node_id}') as the start node.",
                )
                return str(temp_start_node_id)

        logging.error(
            "Could not determine start node ID from the provided JSON data and configuration."
        )
        return None

    def _create_nodes_from_import_data(
        self, loaded_data: Dict[str, Any], determined_start_id: str
    ) -> Dict[str, str]:
        id_map: Dict[str, str] = {}  
        processed_ids: Set[str] = set()
        errors: List[str] = []
        node_id_key = config.get_game_key("node_id")
        char_key = config.get_game_key("character")
        text_key = config.get_game_key("text")
        custom_data_wrapper_key = config.get_game_key("custom_data_wrapper")

        def process_node(key_in_json: str, content: Dict[str, Any]):
            nonlocal processed_ids, id_map, errors
            if not isinstance(content, dict):
                return  

            actual_id_str = str(
                content.get(node_id_key, key_in_json)
            )  
            if not actual_id_str:
                errors.append(
                    f"Entry with key '{key_in_json}' is missing a valid node ID (using key '{node_id_key}'). Skipping."
                )
                return
            if actual_id_str in processed_ids:
                logging.warning(
                    f"Duplicate node ID '{actual_id_str}' encountered (from JSON key '{key_in_json}'). Skipping duplicate."
                )
                return

            char = str(content.get(char_key, ""))
            text = str(content.get(text_key, ""))
            is_start = actual_id_str == determined_start_id

            custom_dict_from_json: Dict[str, Any] = {}
            if custom_data_wrapper_key and custom_data_wrapper_key in content:
                loaded_custom = content[custom_data_wrapper_key]
                if isinstance(loaded_custom, dict):
                    custom_dict_from_json = loaded_custom
                else:
                    logging.warning(
                        f"Custom data wrapper '{custom_data_wrapper_key}' for node '{actual_id_str}' is not a dictionary. Ignoring."
                    )
            elif not custom_data_wrapper_key:  
                for internal_key in config.ALLOWED_CUSTOM_PROPERTIES.keys():
                    game_key_for_prop = config.get_game_key(internal_key)
                    if game_key_for_prop and game_key_for_prop in content:
                        custom_dict_from_json[internal_key] = content[game_key_for_prop]
                    elif (
                        internal_key in content
                    ):  
                        custom_dict_from_json[internal_key] = content[internal_key]

            try:
                node_data = DialogueNodeData(
                    node_id=actual_id_str,
                    character=char,
                    text=text,
                    is_start=is_start,
                    pos=QPointF(
                        50, 50 + len(processed_ids) * 20
                    ),  
                    custom_data=custom_dict_from_json,
                )
                if self._add_node_internal(node_data):
                    processed_ids.add(actual_id_str)
                    id_map[actual_id_str] = key_in_json  
                else:
                    raise ValueError("Internal node addition failed.")
            except Exception as e:
                error_msg = f"Key '{key_in_json}' (ID: '{actual_id_str}'): {e}"
                errors.append(error_msg)
                logging.exception(f"Error creating node from import: {error_msg}")

        start_wrapper_key = config.get_game_key("start_node_wrapper")

        if start_wrapper_key and start_wrapper_key in loaded_data:
            start_content = loaded_data[start_wrapper_key]
            if isinstance(start_content, dict):
                start_node_id_in_wrapper = str(start_content.get(node_id_key))
                if start_node_id_in_wrapper == determined_start_id:
                    if determined_start_id not in processed_ids:
                        logging.debug(
                            f"Processing start node '{determined_start_id}' from within wrapper '{start_wrapper_key}'."
                        )
                        process_node(start_wrapper_key, start_content)
                else:  
                    logging.warning(
                        f"Start wrapper key '{start_wrapper_key}' contained node with ID '{start_node_id_in_wrapper}', but determined start was '{determined_start_id}'. Processing wrapper content as regular node '{start_wrapper_key}'."
                    )
                    process_node(start_wrapper_key, start_content)

        for k, v in loaded_data.items():
            if k == start_wrapper_key:
                continue  
            if isinstance(v, dict):
                node_id_val = str(v.get(node_id_key, k))
                if (
                    node_id_val not in processed_ids
                ):  
                    process_node(k, v)

        if errors:
            QMessageBox.warning(
                self,
                "Import Node Errors",
                f"Errors occurred during node creation from JSON:\n- "
                + "\n- ".join(errors),
            )
        return id_map

    def _set_editor_start_node(self, start_node_id: str) -> bool:
        if not start_node_id:
            logging.error("Start node ID provided to _set_editor_start_node was empty.")
            QMessageBox.critical(
                self,
                "Import Error",
                "Internal error: Start node ID missing during import setup.",
            )
            return False

        gnode = self.graphics_nodes.get(start_node_id)
        if gnode:
            if self._set_start_node_internal(gnode, force=True):
                logging.info(
                    f"Successfully set start node to '{self.start_node_id}' after import."
                )
                return True
            else:
                QMessageBox.critical(
                    self,
                    "Import Error",
                    f"Internal function failed to set start node to '{start_node_id}'.",
                )
                return False
        elif not self.nodes_data:  
            logging.error(
                "Cannot set start node: No valid node data was created during import."
            )
            QMessageBox.critical(
                self,
                "Import Error",
                "Cannot set start node because no valid node data was created during import.",
            )
            return False
        else:  
            logging.error(
                f"Determined start node ID '{start_node_id}' exists in data but not in graphics_nodes."
            )
            QMessageBox.critical(
                self,
                "Import Error",
                f"Internal inconsistency: Start node '{start_node_id}' graphics item not found.",
            )
            logging.debug(
                f"Available graphics nodes: {list(self.graphics_nodes.keys())}"
            )
            return False

    def _populate_connections_from_import_data(
        self, loaded_data: Dict[str, Any], id_to_key_map: Dict[str, str]
    ):
        errors: List[str] = []
        logging.info("Populating node connections from imported JSON...")

        next_k = config.get_game_key("next_node")
        choices_k = config.get_game_key("choices")
        ctext_k = config.get_game_key("choice_text")
        ctgt_k = config.get_game_key("choice_target")
        cicon_k = config.get_game_key("choice_icon")
        csnd_k = config.get_game_key("choice_sound")
        start_wrap_k = config.get_game_key("start_node_wrapper")
        node_id_k = config.get_game_key("node_id")

        for node_id, target_node_data in self.nodes_data.items():
            orig_key = id_to_key_map.get(node_id)
            content: Optional[Dict] = None

            if (
                orig_key
                and orig_key in loaded_data
                and isinstance(loaded_data[orig_key], dict)
            ):
                content = loaded_data[orig_key]
            elif (
                node_id == self.start_node_id
                and start_wrap_k
                and start_wrap_k in loaded_data
            ):  
                wrap_content = loaded_data[start_wrap_k]
                if (
                    isinstance(wrap_content, dict)
                    and str(wrap_content.get(node_id_k)) == node_id
                ):
                    content = wrap_content

            if not content:
                logging.warning(
                    f"Skipping connections for node '{node_id}': Original JSON content not found or not a dict."
                )
                continue

            target_node_data.next_node = None  
            target_node_data.choices = []

            if next_k and next_k in content:
                next_id_val = content[next_k]
                if isinstance(next_id_val, (str, int, float)) and str(next_id_val):
                    next_id_str = str(next_id_val)
                    if next_id_str in self.nodes_data:
                        target_node_data.next_node = next_id_str
                        target_node_data.choices = []  
                        logging.debug(
                            f"  Node '{node_id}': Set next -> '{next_id_str}'"
                        )
                        continue  
                    else:
                        errors.append(
                            f"Node '{node_id}': Target '{next_id_str}' for '{next_k}' key does not exist."
                        )
                elif next_id_val:
                    errors.append(
                        f"Node '{node_id}': Invalid type '{type(next_id_val)}' for '{next_k}' key."
                    )

            elif (
                choices_k
                and choices_k in content
                and isinstance(content[choices_k], list)
            ):
                valid_choices_added = False
                logging.debug(f"  Node '{node_id}': Processing choices list...")
                for i, choice_obj in enumerate(content[choices_k]):
                    if isinstance(choice_obj, dict):
                        ct = choice_obj.get(ctext_k)
                        cn = choice_obj.get(ctgt_k)
                        if ct is None:
                            errors.append(
                                f"Node '{node_id}', Choice {i+1}: Missing choice text key ('{ctext_k}'). Skipping choice."
                            )
                            continue
                        if cn is None:
                            errors.append(
                                f"Node '{node_id}', Choice '{ct}': Missing choice target key ('{ctgt_k}'). Skipping choice."
                            )
                            continue

                        ct_str = str(ct)
                        cn_str = str(cn)

                        if cn_str not in self.nodes_data:
                            errors.append(
                                f"Node '{node_id}', Choice '{ct_str}': Target node '{cn_str}' does not exist. Skipping choice."
                            )
                            continue

                        icon = choice_obj.get(cicon_k)
                        sound = choice_obj.get(csnd_k)
                        preset_name = "None"
                        for name, preset_data in config.PRESETS.items():
                            if name == "None":
                                continue
                            preset_icon = preset_data.get("icon")
                            preset_sound = preset_data.get("sound")

                            icon_match = (preset_icon is None) or (icon == preset_icon)
                            sound_match = (preset_sound is None) or (
                                sound == preset_sound
                            )
                            if icon_match and sound_match:

                                if preset_icon or preset_sound:
                                    preset_name = name
                                    break  
                        target_node_data.choices.append((ct_str, cn_str, preset_name))
                        valid_choices_added = True
                        logging.debug(
                            f"    Added choice: '{ct_str}' -> '{cn_str}' (Preset: {preset_name})"
                        )
                    else:
                        errors.append(
                            f"Node '{node_id}', Choice {i+1}: Item in '{choices_k}' list is not a dictionary."
                        )

                if valid_choices_added:
                    target_node_data.next_node = (
                        None  
                    )

        if errors:
            QMessageBox.warning(
                self,
                "Import Connection Errors",
                f"Issues found while connecting nodes from JSON:\n- "
                + "\n- ".join(errors),
            )

    def export_to_json(self):
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()
        if not self.nodes_data:
            QMessageBox.warning(self, "Export Error", "There are no nodes to export.")
            return
        if not self.start_node_id or self.start_node_id not in self.nodes_data:
            QMessageBox.warning(
                self,
                "Export Error",
                "A valid start node is required for export. Please set one.",
            )
            return

        final_export_start_id = self._ensure_start_node_is_start_for_export()
        if not final_export_start_id:
            logging.warning("Export cancelled due to start node handling.")
            return

        default_name = f"{final_export_start_id}_dialogue.json"
        start_dir = (
            os.path.dirname(self.current_project_path)
            if self.current_project_path
            else ""
        )
        fpath, _ = QFileDialog.getSaveFileName(
            self,
            "Export JSON",
            os.path.join(start_dir, default_name),
            config.GAME_JSON_FILTER,
        )
        if not fpath:
            logging.info("Export cancelled by user.")
            return

        output_dict = self._build_export_dict(final_export_start_id)
        if not output_dict:
            QMessageBox.critical(
                self, "Export Failed", "Failed to build the dictionary for JSON export."
            )
            return

        try:
            logging.info(f"Exporting JSON to: {fpath}")
            with open(fpath, "w", encoding="utf-8") as f:
                json.dump(output_dict, f, indent=config.JSON_INDENT)
            QMessageBox.information(
                self,
                "Export Successful",
                f"Dialogue successfully exported to:\n{fpath}",
            )
        except Exception as e:
            logging.exception(f"An error occurred while writing the JSON file: {e}")
            QMessageBox.critical(
                self,
                "Export Failed",
                f"An error occurred while writing the JSON file:\n{e}",
            )

    def _ensure_start_node_is_start_for_export(self) -> Optional[str]:
        current_start_id = self.start_node_id
        target_id = config.START_NODE_EXPORT_ID

        if not current_start_id:
            logging.error("Cannot ensure start node for export: No start node set.")
            return None
        if current_start_id == target_id:
            return target_id  

        gnode = self.graphics_nodes.get(current_start_id)
        if not gnode:
            logging.critical(
                f"Export Error: Start node '{current_start_id}' exists in data but not graphics!"
            )
            QMessageBox.critical(
                self,
                "Export Error",
                f"Internal error: Start node graphics item missing for '{current_start_id}'.",
            )
            return None

        if (
            target_id in self.nodes_data
            and self.nodes_data[target_id] != gnode.node_data
        ):
            logging.error(
                f"Export Error: Cannot rename start node '{current_start_id}' to '{target_id}' due to ID conflict."
            )
            QMessageBox.critical(
                self,
                "Export Error - ID Conflict",
                f"Cannot automatically rename start node '{current_start_id}' to '{target_id}' for export because another node already uses the ID '{target_id}'.\nPlease resolve the conflict manually before exporting.",
            )
            return None

        reply = QMessageBox.question(
            self,
            "Confirm Rename for Export",
            f"The current start node is '{current_start_id}'. For standard game compatibility, it should usually be named '{target_id}'.\n\nRename node '{current_start_id}' to '{target_id}' for this export?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes,
        )

        if reply == QMessageBox.StandardButton.Yes:
            logging.info(
                f"Pushing command to rename start node '{current_start_id}' to '{target_id}' for export."
            )
            cmd = RenameNodeIdCommand(current_start_id, target_id, self)
            self.undo_stack.push(cmd)

            if self.start_node_id == target_id:
                logging.info(f"Renamed start node to '{target_id}' successfully.")
                return target_id
            else:
                logging.error(
                    f"Rename command failed or did not result in start node being '{target_id}'."
                )
                QMessageBox.critical(
                    self,
                    "Rename Failed",
                    f"Failed to rename start node '{current_start_id}' to '{target_id}'. Export aborted.",
                )
                return None
        else:
            logging.warning(
                f"Exporting with non-standard start node ID '{current_start_id}'."
            )
            QMessageBox.warning(
                self,
                "Export Warning",
                f"Exporting with start node ID '{current_start_id}'.\nThis might not be compatible with the target game if it expects the start node ID to be '{target_id}'.",
            )
            return current_start_id

    def _build_export_dict(
        self, final_export_start_id: str
    ) -> Optional[Dict[str, Any]]:
        output_nodes: Dict[str, Dict] = {}
        start_entry: Optional[Dict] = None
        errors: List[str] = []
        key_map = config.GAME_KEY_MAP
        logging.info(
            f"Building export dictionary with start node '{final_export_start_id}'..."
        )

        try:
            for node_id, data in self.nodes_data.items():

                if node_id in self.graphics_nodes:
                    data.pos = self.graphics_nodes[node_id].pos()

                entry: Dict[str, Any] = {}

                node_id_key = key_map.get("node_id")
                if node_id_key:
                    entry[node_id_key] = data.id
                else:
                    errors.append(
                        f"Missing 'node_id' key in GAME_KEY_MAP. Cannot export node {data.id}."
                    )
                    continue

                char_key = key_map.get("character")
                text_key = key_map.get("text")
                if char_key and data.character:
                    entry[char_key] = data.character
                if text_key and data.text:
                    entry[text_key] = data.text

                has_explicit_connection = False
                next_key = key_map.get("next_node")
                choices_key = key_map.get("choices")
                ctext_key = key_map.get("choice_text")
                ctgt_key = key_map.get("choice_target")
                cicon_key = key_map.get("choice_icon")
                csnd_key = key_map.get("choice_sound")

                if next_key and data.next_node:
                    entry[next_key] = data.next_node
                    has_explicit_connection = True
                    if data.next_node not in self.nodes_data:
                        errors.append(
                            f"Node '{node_id}': Dangling 'next' link to missing node '{data.next_node}'. Exporting link anyway."
                        )
                elif choices_key and data.choices:
                    valid_choices_list = []
                    for ct, tid, pre in data.choices:
                        if tid in self.nodes_data:
                            c_entry: Dict[str, Any] = {}
                            if ctext_key and ct:
                                c_entry[ctext_key] = ct
                            if ctgt_key:
                                c_entry[ctgt_key] = tid
                            else:
                                errors.append(
                                    f"Missing 'choice_target' key in GAME_KEY_MAP. Cannot export choice for {node_id}."
                                )
                                continue

                            if pre != "None":
                                preset_data = config.PRESETS.get(pre)
                                if preset_data:
                                    icon = preset_data.get("icon")
                                    sound = preset_data.get("sound")
                                    if icon and cicon_key:
                                        c_entry[cicon_key] = icon
                                    if sound and csnd_key:
                                        c_entry[csnd_key] = sound
                            valid_choices_list.append(c_entry)
                        else:
                            errors.append(
                                f"Node '{node_id}': Choice '{ct}' links to missing target '{tid}'. Omitting choice."
                            )
                    if valid_choices_list:
                        entry[choices_key] = valid_choices_list
                        has_explicit_connection = True

                if not has_explicit_connection:
                    end_flag_key = key_map.get("end_flag")

                    if data.id == final_export_start_id:
                        if end_flag_key:
                            entry[end_flag_key] = True
                            logging.warning(
                                f"Start node '{data.id}' has no outgoing connections. Marked as end: {bool(end_flag_key)}"
                            )

                    else:
                        if next_key:
                            entry[next_key] = final_export_start_id
                            logging.info(
                                f"Node '{data.id}' has no connections. Adding implicit link to start node '{final_export_start_id}'."
                            )
                        else:
                            errors.append(
                                f"Node '{data.id}' has no connections, but 'next_node' key missing in GAME_KEY_MAP. Cannot add implicit link."
                            )

                custom_data_wrapper_key = key_map.get("custom_data_wrapper")
                props_to_export: Dict[str, Any] = {}
                if isinstance(data.custom_data, dict):
                    for internal_key, value in data.custom_data.items():
                        if (
                            internal_key in config.ALLOWED_CUSTOM_PROPERTIES
                            and value is not None
                        ):

                            game_key_for_prop = key_map.get(
                                internal_key
                            )  
                            if game_key_for_prop:
                                props_to_export[game_key_for_prop] = value
                            elif custom_data_wrapper_key:
                                props_to_export[internal_key] = (
                                    value  
                                )
                            else:
                                errors.append(
                                    f"Node '{node_id}': No game key mapping for custom prop '{internal_key}' and no wrapper. Skipping property."
                                )
                if props_to_export:
                    if custom_data_wrapper_key:
                        entry[custom_data_wrapper_key] = props_to_export
                    else:  
                        for prop_key, prop_value in props_to_export.items():
                            if prop_key in entry:
                                errors.append(
                                    f"Node '{node_id}': Custom prop game key '{prop_key}' conflicts with standard key. Skipping property."
                                )
                            else:
                                entry[prop_key] = prop_value

                if data.id == final_export_start_id:
                    start_entry = entry
                else:
                    output_nodes[data.id] = entry

            if not start_entry:
                raise ValueError(
                    f"Start node entry for '{final_export_start_id}' was not created during export build."
                )

            start_wrapper_key = key_map.get(
                "start_node_wrapper", "start"
            )  
            final_output: Dict[str, Any] = {start_wrapper_key: start_entry}

            error_node_config = getattr(config, "DEFAULT_ERROR_NODE", None)
            error_wrapper_key = key_map.get("error_node_wrapper")
            error_node_id_key = key_map.get("node_id")
            if error_node_config and error_wrapper_key and error_node_id_key:
                error_node_id = error_node_config.get(error_node_id_key)
                if error_node_id:
                    if error_node_id in output_nodes:
                        final_output[error_wrapper_key] = output_nodes.pop(
                            error_node_id
                        )
                        logging.debug(
                            f"Moved processed node '{error_node_id}' to error wrapper '{error_wrapper_key}'."
                        )
                    elif error_node_id == final_export_start_id:
                        final_output[error_wrapper_key] = (
                            start_entry  
                        )
                        logging.warning(
                            f"Start node '{error_node_id}' is also designated error node."
                        )
                    else:  
                        logging.warning(
                            f"Error node '{error_node_id}' not found in dialogue, adding default error node structure."
                        )
                        final_output[error_wrapper_key] = error_node_config.copy()

            final_output.update(sorted(output_nodes.items()))

            if errors:
                QMessageBox.warning(
                    self,
                    "Export Warnings",
                    f"Potential issues found during export build:\n- "
                    + "\n- ".join(errors),
                )
            return final_output

        except Exception as e:
            logging.exception(f"Error building export dictionary: {e}")
            QMessageBox.critical(
                self,
                "Export Build Error",
                f"An internal error occurred while building the export data:\n{e}",
            )
            return None

    def update_bookmark_menu(self):
        if not self.bookmarks_menu:
            return
        self.bookmarks_menu.clear()
        if not self.bookmarks:
            no_bookmarks_action = QAction("No Bookmarks", self)
            no_bookmarks_action.setEnabled(False)
            self.bookmarks_menu.addAction(no_bookmarks_action)
            return

        for node_id in sorted(list(self.bookmarks)):
            if node_id in self.nodes_data:
                node_data = self.nodes_data[node_id]
                label = f"{node_id}: {node_data.character} - {node_data.text[:20]}..."
                action = QAction(label, self)

                action.triggered.connect(partial(self.handle_focus_request, node_id))
                self.bookmarks_menu.addAction(action)
            else:  
                logging.warning(f"Stale bookmark found for missing node ID: {node_id}")

    def update_dynamic_node_visuals(self, node_id: Optional[str] = None):
        """Updates visuals like bookmark status and color rules for one or all nodes."""
        nodes_to_update = []
        if node_id and node_id in self.graphics_nodes:
            nodes_to_update.append(self.graphics_nodes[node_id])
        elif node_id is None:  
            nodes_to_update = list(self.graphics_nodes.values())

        for gnode in nodes_to_update:
            if hasattr(gnode, "update_dynamic_visuals"):
                gnode.update_dynamic_visuals()

    def update_all_dynamic_node_visuals(self):
        self.update_dynamic_node_visuals(None)  

    def copy_selected_nodes(self):
        selected_nodes = self.get_selected_nodes()
        if not selected_nodes:
            return

        self.clipboard.clear()
        for node in selected_nodes:
            if hasattr(node, "node_data"):

                self.clipboard.append(copy.deepcopy(node.node_data.to_dict()))
        self.statusBar().showMessage(f"Copied {len(self.clipboard)} node(s).", 2000)

    def has_clipboard_data(self) -> bool:
        return bool(self.clipboard)

    def paste_nodes(self, paste_center_pos: Optional[QPointF] = None):
        if not self.clipboard:
            return
        self._push_text_edit_command()
        self._push_char_edit_command()
        self._check_pending_custom_prop_changes()

        if paste_center_pos is None:
            paste_center_pos = (
                self.view.mapToScene(self.view.viewport().rect().center())
                if self.view
                else QPointF(100, 100)
            )

        self.undo_stack.beginMacro(f"Paste {len(self.clipboard)} Node(s)")
        try:
            avg_original_x = 0.0
            avg_original_y = 0.0
            num_nodes = 0
            for node_data_dict in self.clipboard:
                pos_list = node_data_dict.get(config.get_project_key("pos"))
                if isinstance(pos_list, list) and len(pos_list) == 2:
                    avg_original_x += pos_list[0]
                    avg_original_y += pos_list[1]
                    num_nodes += 1
            if num_nodes > 0:
                avg_original_x /= num_nodes
                avg_original_y /= num_nodes
            else:  
                avg_original_x = paste_center_pos.x() - config.NODE_WIDTH / 2
                avg_original_y = paste_center_pos.y() - config.NODE_HEIGHT / 2

            offset_x = (
                paste_center_pos.x()
                - avg_original_x
                - (config.NODE_WIDTH / 2 if num_nodes > 0 else 0)
            )
            offset_y = (
                paste_center_pos.y()
                - avg_original_y
                - (config.NODE_HEIGHT / 2 if num_nodes > 0 else 0)
            )
            if num_nodes <= 1:
                offset_y += (
                    config.DEFAULT_PASTE_OFFSET.y()
                )  

            pasted_ids_map = {}  

            for i, node_data_dict in enumerate(self.clipboard):
                original_id = node_data_dict.get(config.get_project_key("id"))
                new_id = self._get_unique_node_id(
                    base=f"{original_id}_copy_" if original_id else "pasted_node_"
                )

                node_data_dict[config.get_project_key("id")] = new_id
                node_data_dict[config.get_project_key("is_start_node")] = (
                    False  
                )

                pos_list = node_data_dict.get(config.get_project_key("pos"))
                if isinstance(pos_list, list) and len(pos_list) == 2:
                    new_x = pos_list[0] + offset_x
                    new_y = pos_list[1] + offset_y
                    node_data_dict[config.get_project_key("pos")] = [new_x, new_y]
                else:  
                    node_data_dict[config.get_project_key("pos")] = [
                        paste_center_pos.x() - config.NODE_WIDTH / 2 + (i * 10),
                        paste_center_pos.y()
                        - config.NODE_HEIGHT / 2
                        + (i * 10)
                        + config.DEFAULT_PASTE_OFFSET.y(),
                    ]

                node_data_dict.pop(config.get_project_key("next_node"), None)
                node_data_dict.pop(config.get_project_key("choices"), None)

                try:
                    new_node_data_obj = DialogueNodeData.from_dict(node_data_dict)

                    cmd = AddNodeCommand(new_node_data_obj, self)
                    self.undo_stack.push(cmd)  
                    if original_id:
                        pasted_ids_map[original_id] = (
                            new_id  
                        )
                except Exception as e:
                    logging.error(
                        f"Error creating pasted node from data {node_data_dict}: {e}"
                    )
                    QMessageBox.warning(
                        self,
                        "Paste Error",
                        f"Failed to create pasted node for original ID '{original_id}'.",
                    )
                    self.undo_stack.endMacro()  
                    return

        except Exception as e:
            logging.exception(f"Error during node pasting process: {e}")
            QMessageBox.critical(
                self, "Paste Failed", f"An unexpected error occurred during paste:\n{e}"
            )
        finally:
            self.undo_stack.endMacro()

    def highlight_path(self, start_node_id: str, incoming: bool = False):
        self.clear_all_highlights()
        if not start_node_id or start_node_id not in self.graphics_nodes:
            return

        q = deque([start_node_id])
        visited_nodes = {start_node_id}
        highlighted_edges = set()

        adj = self.get_graph_adjacency(incoming=incoming, exclude_start_loops=True)

        start_gnode = self.graphics_nodes.get(start_node_id)
        if start_gnode:
            if incoming:
                start_gnode.set_path_highlight_incoming(True)
            else:
                start_gnode.set_path_highlight_outgoing(True)

        while q:
            u_id = q.popleft()
            u_gnode = self.graphics_nodes.get(u_id)

            neighbors = adj.get(u_id, [])
            for v_id in neighbors:
                v_gnode = self.graphics_nodes.get(v_id)
                if not v_gnode:
                    continue

                edge_found = False

                src_node = v_gnode if incoming else u_gnode
                dest_node = u_gnode if incoming else v_gnode

                for edge in self.graphics_edges:
                    if edge.source == src_node and edge.dest == dest_node:
                        if edge not in highlighted_edges:
                            edge.set_path_highlight(True)
                            highlighted_edges.add(edge)
                            edge_found = True

                if v_id not in visited_nodes:
                    visited_nodes.add(v_id)
                    q.append(v_id)
                    if incoming:
                        v_gnode.set_path_highlight_incoming(True)
                    else:
                        v_gnode.set_path_highlight_outgoing(True)

    def highlight_outgoing_path(self, node_id: Optional[str] = None):
        if node_id is None:
            selected_node = self.get_selected_node()
            if selected_node:
                node_id = selected_node.node_data.id
            else:
                return  
        logging.info(f"Highlighting outgoing path from: {node_id}")
        self.highlight_path(node_id, incoming=False)

    def highlight_incoming_path(self, node_id: Optional[str] = None):
        if node_id is None:
            selected_node = self.get_selected_node()
            if selected_node:
                node_id = selected_node.node_data.id
            else:
                return
        logging.info(f"Highlighting incoming path to: {node_id}")
        self.highlight_path(node_id, incoming=True)

    def clear_all_highlights(self):
        logging.debug("Clearing all highlights.")
        for node in self.graphics_nodes.values():
            node.set_path_highlight_outgoing(False)
            node.set_path_highlight_incoming(False)
            node.set_find_highlight(False)
        for edge in self.graphics_edges:
            edge.set_path_highlight(False)
            edge.set_find_highlight(False)

        self._find_results = []
        self._find_index = -1
        self._find_term = ""
        self._find_type = ""

    def get_graph_adjacency(
        self, incoming: bool = False, exclude_start_loops: bool = False
    ) -> Dict[str, List[str]]:
        adj = defaultdict(list)
        start_id = self.start_node_id  

        for node_id, node_data in self.nodes_data.items():
            targets = []

            if node_data.next_node and node_data.next_node in self.nodes_data:

                if not exclude_start_loops or node_data.next_node != start_id:
                    targets.append(node_data.next_node)
            elif node_data.choices:
                for _, target_id, _ in node_data.choices:
                    if target_id in self.nodes_data:

                        if not exclude_start_loops or target_id != start_id:
                            targets.append(target_id)

            if (
                not targets
                and node_id != start_id
                and start_id in self.nodes_data
                and not exclude_start_loops
            ):
                targets.append(start_id)

            for target_id in targets:
                if incoming:
                    adj[target_id].append(node_id)
                else:
                    adj[node_id].append(target_id)

        return adj

    def validate_dialogue(self):
        results: List[Tuple[str, str, Optional[str]]] = []  
        start_id = self.start_node_id

        if not start_id or start_id not in self.nodes_data:
            results.append(("ERROR", "No valid start node defined.", None))

        adj = self.get_graph_adjacency(incoming=False)
        rev_adj = self.get_graph_adjacency(incoming=True)
        all_node_ids = set(self.nodes_data.keys())

        reachable_nodes = set()
        if start_id and start_id in all_node_ids:
            q = deque([start_id])
            reachable_nodes.add(start_id)
            while q:
                u = q.popleft()
                for v in adj.get(u, []):
                    if v not in reachable_nodes:
                        reachable_nodes.add(v)
                        q.append(v)
            if config.VALIDATOR_CHECK_ORPHANS:
                orphans = all_node_ids - reachable_nodes
                for orphan_id in sorted(list(orphans)):
                    results.append(
                        (
                            "WARNING",
                            f"Node is unreachable from the start node ('{start_id}').",
                            orphan_id,
                        )
                    )

        for node_id, node_data in self.nodes_data.items():
            has_outgoing = False

            if node_data.next_node:
                if (
                    config.VALIDATOR_CHECK_DANGLING_LINKS
                    and node_data.next_node not in all_node_ids
                ):
                    results.append(
                        (
                            "ERROR",
                            f"Links to non-existent node ID '{node_data.next_node}' via 'Next'.",
                            node_id,
                        )
                    )
                else:
                    has_outgoing = True

            elif node_data.choices:
                has_outgoing = (
                    True  
                )
                for i, (text, target_id, preset) in enumerate(node_data.choices):
                    if (
                        config.VALIDATOR_CHECK_DANGLING_LINKS
                        and target_id not in all_node_ids
                    ):
                        results.append(
                            (
                                "ERROR",
                                f"Choice {i+1} ('{text[:20]}...') links to non-existent node ID '{target_id}'.",
                                node_id,
                            )
                        )
                        has_outgoing = False  

            if config.VALIDATOR_CHECK_DEAD_ENDS and not has_outgoing:
                is_looping_to_start = False
                if (
                    node_id != start_id and start_id in all_node_ids
                ):  
                    is_looping_to_start = True

                if not is_looping_to_start:
                    results.append(
                        (
                            "WARNING",
                            "Node has no outgoing connections (potential dead end).",
                            node_id,
                        )
                    )

            if config.VALIDATOR_CHECK_MISSING_TEXT and not node_data.text.strip():
                results.append(("WARNING", "Node text is empty.", node_id))
            if (
                config.VALIDATOR_CHECK_MISSING_CHARACTER
                and not node_data.character.strip()
            ):
                results.append(("WARNING", "Node character is empty.", node_id))

        if not results:
            QMessageBox.information(self, "Validation Complete", "No issues found.")
        else:
            dialog = ValidationResultsDialog(results, self)
            dialog.focusRequested.connect(self.handle_focus_request)  
            dialog.exec()

if __name__ == "__main__":
    try:
        if hasattr(Qt.ApplicationAttribute, "AA_EnableHighDpiScaling"):
            QApplication.setAttribute(
                Qt.ApplicationAttribute.AA_EnableHighDpiScaling, True
            )
            logging.info("High DPI Scaling enabled.")
        if hasattr(Qt.ApplicationAttribute, "AA_UseHighDpiPixmaps"):
            QApplication.setAttribute(
                Qt.ApplicationAttribute.AA_UseHighDpiPixmaps, True
            )
            logging.info("High DPI Pixmaps enabled.")
    except Exception as e:
        logging.warning(f"Could not set High DPI attributes: {e}")

    app = QApplication(sys.argv)

    app_name = getattr(config, "APP_NAME", "Dialogue Editor")
    app_version = getattr(config, "APP_VERSION", "1.0")
    org_name = getattr(config, "ORG_NAME", None)
    org_domain = getattr(config, "ORG_DOMAIN", None)
    if org_name:
        app.setOrganizationName(org_name)
    if org_domain:
        app.setOrganizationDomain(org_domain)
    app.setApplicationName(app_name)
    app.setApplicationVersion(app_version)

    editor = DialogueEditor()
    editor.show()

    if not editor.nodes_data:
        logging.info("No nodes found on startup. Creating default 'start' node.")
        editor.add_node_cmd(
            node_id=config.START_NODE_EXPORT_ID,
            character="Narrator",
            text="Start here.",
            pos=QPointF(50, 50),
        )
        editor.undo_stack.clear()  
        editor._mark_unsaved(False)  

    sys.exit(app.exec())
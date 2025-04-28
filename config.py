from PyQt6.QtGui import QColor
from PyQt6.QtCore import QPointF, QSizeF
from typing import Optional, List, Dict, Any
# --- Constants ---
CONN_TYPE_CHOICE = "CHOICE"
CONN_TYPE_NEXT = "NEXT"
CONN_TYPE_IMPLICIT_LOOP = "IMPLICIT_LOOP"
INTERNAL_KEY_PROP = "internal_key" # For custom property widgets
DUMMY_NODE_PREFIX = "__dummy_"

# --- Application Identification ---
APP_NAME = "Ford Dialogue Node Editor"
APP_VERSION = "0.70" 
# ORG_NAME = "YourOrganization" # Optional: For QSettings path
# ORG_DOMAIN = "shit.poster" # Optional: For QSettings path

# --- Custom Properties Configuration ---
# Define allowed custom properties here if you want them editable in the panel
# If empty, the "Custom Properties" section will just be empty.
# Structure:
# "internal_key_name": {
#     "display": "User-Friendly Name", # How it appears in the UI
#     "type": "string" | "int" | "float" | "bool", # Data type for validation/conversion
#     "default": default_value # Default value when creating a node
# }
ALLOWED_CUSTOM_PROPERTIES = {
    # Example:
    # "audio_file": {
    #     "display": "Audio File",
    #     "type": "string",
    #     "default": ""
    # },
    # "quest_id": {
    #     "display": "Quest ID",
    #     "type": "int",
    #     "default": 0
    # },
    # "is_important": {
    #     "display": "Important Dialogue",
    #     "type": "bool",
    #     "default": False
    # }
}

# --- Key Mapping Configuration ---
# Defines how internal node attributes map to keys in the PROJECT save file (.dveditor)
# This allows changing internal variable names without breaking project files (mostly).
PROJECT_KEY_MAP = {
    # Core Data
    "id": "id",
    "character": "character",
    "text": "text",
    "choices": "choices",
    "next_node": "next_node",
    "pos": "pos",
    "is_start_node": "is_start_node",
    "custom_data": "custom_data", # Key for the dictionary holding ALL custom properties
    # Editor Specific Data
    "bookmarks": "bookmarks",         # Key for the list of bookmarked node IDs
    "visual_groups": "visual_groups"  # Key for the list of visual group data dicts
}
# Automatically create the reverse mapping for loading project files
PROJECT_REVERSE_KEY_MAP = {v: k for k, v in PROJECT_KEY_MAP.items()}

# Defines how internal node attributes/concepts map to keys in the GAME JSON export/import
# This allows the editor's internal names to differ from the final game format.
GAME_KEY_MAP = {
    # Core node attributes used in game
    "node_id": "id",         # Internal concept 'node_id' maps to game JSON key 'id'
    "character": "character",
    "text": "text",
    # Connections
    "next_node": "next",         # Internal concept 'next_node' maps to game JSON key 'next'
    "choices": "choices",        # Internal concept 'choices' (the list) maps to 'choices'
    "end_flag": "end",           # Optional: A boolean flag added to nodes with no outgoing links (except implicit start loops)
    # Keys within each choice object in the 'choices' list
    "choice_text": "text",       # Text displayed for the choice
    "choice_target": "next",     # Target node ID the choice leads to
    # Optional preset keys within a choice object (used if PRESETS are defined)
    "choice_icon": "icon",       # Key for icon name/path in game JSON choice object
    "choice_sound": "sound",     # Key for sound name/path in game JSON choice object
    # Special handling keys for the root of the game JSON
    "start_node_wrapper": "start",   # Key in the root whose value is the start node object/ID
    "error_node_wrapper": "error",   # Optional: Key in the root for a default error node object
    # Handling of custom properties from ALLOWED_CUSTOM_PROPERTIES during export/import
    "custom_data_wrapper": None,     # Optional: If game JSON nests custom props under a specific key, specify it here (e.g., "properties").
                                     # If None, custom props are exported flatly into the main node object using their mapped game keys below.
    # --- Mappings for your Custom Properties (Required if exporting/importing custom props) ---
    # Map internal keys from ALLOWED_CUSTOM_PROPERTIES to their desired game JSON key.
    # Required if custom_data_wrapper is None, or if importing custom props even with a wrapper.
    # Example:
    # "audio_file": "audio", # Internal "audio_file" maps to "audio" in game JSON
    # "quest_id": "questId", # Internal "quest_id" maps to "questId" in game JSON
    # "is_important": "isImportant",
}

# --- Helper Functions (Define BEFORE use in DEFAULT_ERROR_NODE) ---
def get_project_key(internal_key: str) -> str:
    """Safely gets the project file key for an internal attribute name."""
    return PROJECT_KEY_MAP.get(internal_key, internal_key) # Fallback to internal name if not mapped

def get_game_key(internal_concept_key: str) -> Optional[str]:
    """Safely gets the game export/import key for an internal concept/attribute name."""
    # Returns None if the key isn't defined in GAME_KEY_MAP
    return GAME_KEY_MAP.get(internal_concept_key)

# --- Visuals & Colors ---
NODE_NORMAL_COLOR = QColor("#444455")
NODE_START_COLOR = QColor("#2E8B57")         # SeaGreen for start node
NODE_END_COLOR = QColor("#A52A2A")           # Brown for true dead-end nodes
NODE_LOOP_COLOR = QColor("#4682B4")          # SteelBlue for nodes looping back to start
NODE_SELECTED_BRIGHTNESS = 130               # How much lighter selected nodes are (100 = no change)
NODE_TEXT_COLOR = QColor(230, 230, 230)      # Light gray text on nodes
NODE_BOOKMARK_BORDER_COLOR = QColor(255, 215, 0, 200) # Gold, slightly transparent border for bookmarked nodes
NODE_HIGHLIGHT_OUTLINE_COLOR = QColor(0, 255, 0, 220) # Bright green outline for path highlighting
NODE_HIGHLIGHT_FILL_ALPHA = 30               # Slightly fill highlighted nodes with outline color
NODE_FIND_HIGHLIGHT_COLOR = QColor(0, 191, 255, 220) # Deep sky blue outline for find results

VISUAL_GROUP_DEFAULT_COLOR = QColor(70, 70, 70, 100)   # Semi-transparent dark gray for visual groups
VISUAL_GROUP_SELECTED_COLOR = QColor(90, 90, 90, 150)  # Slightly lighter/more opaque when selected
VISUAL_GROUP_TEXT_COLOR = QColor(200, 200, 200)    # Text color for group labels

FOCUS_FLASH_COLOR = QColor(255, 255, 153, 200) # Yellowish flash when focusing node
NEXT_LINK_TEXT_COLOR = QColor("#7FFFD4")       # Aquamarine for "[NEXT] -> ..." in properties list
DEFAULT_CHOICE_TEXT_COLOR = QColor(230, 230, 230) # Standard text color for choices in properties list
MISSING_LINK_TEXT_COLOR = QColor("orange")     # For "(Missing!)" text in properties list
EDGE_DEFAULT_COLOR = QColor("#BBB")            # Default edge color
EDGE_SELECTED_COLOR = QColor("#F0E68C")        # Khaki/Gold for selected edges
EDGE_DRAG_COLOR = QColor("orange")             # Color of the temporary line when dragging an edge
EDGE_HIGHLIGHT_COLOR = QColor(0, 255, 0, 255)    # Bright green for highlighted edges (path finding)
EDGE_FIND_HIGHLIGHT_COLOR = QColor(0, 191, 255, 255) # Deep sky blue for edges connected to find results
SCENE_BACKGROUND_COLOR = QColor("#333")        # Dark gray background for the scene

# --- Node Coloring Rules (Optional) ---
# Define rules to automatically color nodes based on their character property.
# The key should be the exact character name. The value is the QColor.
# If a character matches a rule, it overrides the default/start/end/loop colors.
NODE_COLORING_RULES: Dict[str, QColor] = {
    # Example:
    # "Narrator": QColor("#666666"),
    # "Player": QColor("#5577AA"),
    # "Guard": QColor("#AA5555"),
}

# --- Dimensions & Layout ---
NODE_WIDTH = 180
NODE_HEIGHT = 100
NODE_V_SPACING = 50                        # Vertical gap used when auto-placing new nodes created via dialog
EDGE_PEN_WIDTH = 1.7
ARROW_SIZE = 8.0                           # Size of the arrowhead points
NODE_BOOKMARK_BORDER_WIDTH = 2.5           # Width of the dashed border for bookmarks
HIGHLIGHT_PEN_WIDTH = 2.5                  # Width of the outline for highlighted nodes/edges

LAYOUT_LAYER_V_GAP = NODE_HEIGHT + 80      # Sugiyama layout: vertical gap between layers
LAYOUT_NODE_H_GAP = 60                     # Sugiyama layout: horizontal gap between node *centers*
LAYOUT_ORDERING_ITERATIONS = 16            # Sugiyama layout: Max iterations for crossing reduction
LAYOUT_DUMMY_WIDTH = 10                    # Sugiyama layout: Width assumed for dummy nodes
LAYOUT_NUDGE_FACTOR = 0.3                  # Sugiyama layout: How strongly nodes are nudged towards successors (0 to 1)
LAYOUT_NUDGE_ITERATIONS = 5                # Sugiyama layout: Max iterations for nudging phase
VIEW_ZOOM_FACTOR = 1.15                    # Zoom factor per mouse wheel step
VIEW_MIN_ZOOM = 0.1
VIEW_MAX_ZOOM = 30.0
VIEW_FIT_PADDING_X = 100.0                 # Padding added around content when fitting view
VIEW_FIT_PADDING_Y = 100.0
DEFAULT_NODE_POS_OFFSET = QPointF(0, NODE_HEIGHT + NODE_V_SPACING) # Default offset for new nodes created via connection dialog (below source)
DEFAULT_PASTE_OFFSET = QPointF(30, 30)     # Offset applied when pasting nodes
DEFAULT_GROUP_SIZE = QSizeF(NODE_WIDTH * 2 + LAYOUT_NODE_H_GAP, NODE_HEIGHT * 2 + LAYOUT_LAYER_V_GAP) # Initial size for new visual groups

# --- Behavior ---
FOCUS_FLASH_DURATION_MS = 350
TEXT_EDIT_SAVE_DELAY_MS = 600              # Debounce delay (ms) for text edits before saving to undo stack
LIST_FOCUS_CLICK_THRESHOLD_PERCENT = 60    # Click right (100-X)% of list item to focus target node
START_NODE_EXPORT_ID = "start"             # Expected node ID for the start node in game export/import by default

# --- File Settings ---
PROJECT_FILE_EXTENSION = ".dveditor"
PROJECT_FILE_FILTER = f"Dialogue Editor Files (*{PROJECT_FILE_EXTENSION});;All Files (*)"
GAME_JSON_FILTER = "JSON Files (*.json);;All Files (*)"
JSON_INDENT = 2                            # Indentation spaces for saved JSON files (project and game)

# --- Validation Settings ---
VALIDATOR_CHECK_ORPHANS = True             # Check for nodes unreachable from the start node
VALIDATOR_CHECK_DEAD_ENDS = True           # Check for nodes with no outgoing links (ignores implicit loops to start)
VALIDATOR_CHECK_DANGLING_LINKS = True      # Check for choices/next links pointing to non-existent node IDs
VALIDATOR_CHECK_MISSING_TEXT = True        # Check for nodes with empty text fields
VALIDATOR_CHECK_MISSING_CHARACTER = False  # Check for nodes with empty character fields (set to True if required)

# --- Presets ---
# Define UI appearance/sound hints for different choice types used during EXPORT.
# The editor uses these to map back from game JSON icon/sound keys to a preset name.
# If choice_icon/choice_sound keys are not defined in GAME_KEY_MAP, this is ignored.
PRESETS = {
    "None": {}, # Default, no special indicators exported
    "Contract": {"icon": "contract_icon", "sound": "ui_contract.wav"},
    "Shop": {"icon": "shop_icon", "sound": "ui_shop.wav"},
}

# --- Defaults ---
# Internal marker value used by AddChoiceCommand. If choice text matches this value
# (e.g., when user leaves text blank during node linking), AddChoiceCommand creates
# a 'next_node' link instead of a choice link. Set to None if you want empty choices allowed.
DEFAULT_CHOICE_TEXT = ""

# --- Default Error Node (Define AFTER helper functions) ---
# Structure used during EXPORT if the error node (defined by error_node_wrapper key)
# doesn't exist in the dialogue. Uses helper functions to get the correct GAME keys.
_error_node_structure = {}
_node_id_game_key = get_game_key("node_id")
_char_game_key = get_game_key("character")
_text_game_key = get_game_key("text")
_end_flag_game_key = get_game_key("end_flag")

if _node_id_game_key: _error_node_structure[_node_id_game_key] = "error_node" # Default ID for error node
if _char_game_key: _error_node_structure[_char_game_key] = "System"
if _text_game_key: _error_node_structure[_text_game_key] = "Error: An invalid dialogue node was reached."
if _end_flag_game_key: _error_node_structure[_end_flag_game_key] = True # Error node should usually be terminal

# Add default custom properties if needed for the error node, using get_game_key("internal_prop_name")
# Example: if GAME_KEY_MAP has "audio_file": "audio"
# _audio_game_key = get_game_key("audio_file")
# if _audio_game_key: _error_node_structure[_audio_game_key] = "sfx_error.wav"

DEFAULT_ERROR_NODE = _error_node_structure if _error_node_structure else None # Set to None if essential keys are missing

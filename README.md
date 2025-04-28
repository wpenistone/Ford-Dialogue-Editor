# Ford Dialogue Node Editor v0.70

A visual editor for creating and managing node-based branching dialogues, built with Python and PyQt6. Designed for game developers, writers, or anyone needing to structure conversations visually with data.

*Originally created for the [NORTHWIND wiki](https://northwinds.fandom.com/wiki/Northwind_Wiki:Home) project.*

![Screenshot](https://github.com/user-attachments/assets/4a19e9f5-4fd3-41f4-90f2-47fb41e56605)

## Features

*   **Visual Node Editing:**
    *   Create, delete, and move nodes freely within the scene.
    *   Nodes display ID, Character (optional), and a preview of the dialogue Text.
    *   Optional automatic node coloring based on character (configurable in `config.py`).
    *   Double-click a node to quickly focus the text editor in the properties panel.
*   **Connections:**
    *   Link nodes using either a single "NEXT Node" (linear progression) or multiple "Choices".
    *   Create choice links by selecting two nodes (Ctrl+L) or by dragging from a node (press 'L' key with one node selected).
    *   Create new target nodes directly from the linking dialogs.
    *   Visual edge rendering using Bezier curves with arrowheads.
    *   Automatic highlighting of edges connected to nodes found via search or path highlighting.
*   **Properties Panel:**
    *   Edit selected node's ID, Character, and full Text content.
    *   View and manage connections ("Next Node" or "Choices").
    *   Add, Edit, and Remove choices.
    *   Double-click `[NEXT]` in the connection list to convert it to a choice (prompts for text/preset).
    *   Double-click choice text in the list to edit the choice.
    *   Click the target ID area (right side) in the list to focus the view on the target node.
    *   Context menu (Right-click) on list items for actions (Remove, Edit, Focus, Clear Next).
    *   Define and edit custom properties for nodes (configurable in `config.py`).
*   **Graph Organization & Navigation:**
    *   **Bookmarks:** Add/remove bookmarks for important nodes via context menu or `View -> Bookmarks` menu for quick navigation. Bookmarked nodes have a visual indicator.
    *   **Visual Groups:** Create labeled, colored background rectangles (Right-click canvas -> "Add Visual Group Here") to visually organize sections of your graph. Saved with the project.
    *   **Path Highlighting:** Right-click node -> "Highlight Outgoing/Incoming Path" to visualize node reachability (ignores loops back to start). Clear highlights via `View -> Clear Highlights` or `Esc`.
    *   **Find:** "Find Node" functionality (`Ctrl+F`) to search by ID, Character, or Text. Highlights all matching nodes and connected edges. Use "Find Next" (`Ctrl+G`) to cycle focus through results.
    *   Pan the scene using middle-mouse drag.
    *   Zoom using the mouse wheel.
    *   "Fit View to Content" (`Ctrl+F`, menu: `View`) to zoom/pan appropriately.
    *   Automatic focus flash on nodes when jumped to via search, list click, or bookmarks.
*   **Editing Utilities:**
    *   **Copy/Paste Nodes:** Copy selected nodes (`Ctrl+C`) and paste them (`Ctrl+V`) onto the canvas. Pasted nodes get new unique IDs and have connections cleared. Relative positions are maintained.
    *   **Undo/Redo:** Robust Undo/Redo system (`Ctrl+Z` / `Ctrl+Y`) for most actions (node/edge/group/bookmark add/delete, property changes, moves, linking, layout).
*   **Layout:**
    *   Apply an automatic Sugiyama-based layered graph layout (`Ctrl+R`) to arrange nodes based on connections.
*   **Validation:**
    *   Run `File -> Validate Dialogue...` to check for common issues like orphan nodes, dead ends, dangling links (to non-existent nodes), and optionally empty text/character fields. Results shown in a clickable list.
*   **Data Management:**
    *   Save/Load projects in a dedicated `.dveditor` JSON format (includes node data, positions, connections, bookmarks, visual groups).
    *   Import/Export dialogues to/from a game-specific JSON format (structure highly configurable via `config.py`).
    *   Automatic prompts to save unsaved changes on exit or load.
*   **Configuration (`config.py`):**
    *   Customize UI colors, node/edge dimensions, layout parameters.
    *   Define **Custom Properties** available for nodes.
    *   Define **Node Coloring Rules** based on character names.
    *   Configure **Key Mappings** for project files and JSON import/export.
    *   Set up **Choice Presets** (e.g., for icons/sounds linked to choices in export).
    *   Configure **Validation Checks**.
*   **Keyboard Shortcuts:**
    *   `Ctrl+N`: Add Node
    *   `Ctrl+O`: Open Project
    *   `Ctrl+S`: Save Project
    *   `Ctrl+Shift+S`: Save Project As...
    *   `Ctrl+C`: Copy Selected Node(s)
    *   `Ctrl+V`: Paste Node(s)
    *   `Del`/`Backspace`: Delete Selected Item(s)
    *   `Ctrl+L`: Link Selected Nodes (requires 2 nodes)
    *   `L` (with 1 node selected): Start drawing an edge
    *   `Esc`: Cancel edge draw / Clear Highlights / Clear Selection
    *   `Ctrl+R`: Apply Auto-Layout
    *   `Ctrl+F`: Find Node
    *   `Ctrl+G`: Find Next Result
    *   `Ctrl+Z`: Undo
    *   `Ctrl+Y` / `Ctrl+Shift+Z`: Redo

## Installation

1.  **Prerequisites:**
    *   Python 3.x (developed with 3.9+, tested minimally on 3.13, should work on most 3.x)
2.  **Download:** Obtain the `FordDialogueEditor.py` and `config.py` files. Place them in the same directory.
3.  **Install Dependencies:**
    *   The primary dependency is PyQt6. Open a terminal or command prompt in the directory where you saved the files and run:
    ```bash
    pip install PyQt6
    ```

## Usage

1.  **Run the Editor:**
    ```bash
    python FordDialogueEditor.py
    ```
    (Make sure your terminal's current directory is where the `.py` file is located).
2.  **Creating Nodes:**
    *   Right-click on the empty canvas -> "Add Node Here".
    *   Use the `Edit -> Add Node` menu item (`Ctrl+N`).
    *   A default "start" node is created if the editor is launched empty.
3.  **Selecting:**
    *   Click on a node, edge, or visual group to select it.
    *   Hold `Ctrl` and click to select multiple items.
    *   Drag a selection box to select multiple items.
4.  **Editing Properties:**
    *   Select a single node to view and edit its properties (ID, Character, Text, Custom Props, Connections) in the right-hand panel.
    *   Text/Character edits are saved automatically after a short delay (debounced).
    *   ID changes require pressing Enter or focus leaving the input field. The `start` node ID cannot be changed here (use Set as Start action).
    *   Double-click a node in the scene to focus the Text editor in the panel.
5.  **Connecting Nodes:**
    *   **Choices:**
        *   Select two nodes, then use `Edit -> Link Selected Nodes` (`Ctrl+L`). You'll be prompted for choice text and preset.
        *   Select one source node, press `L`, then click-and-drag to the target node. Release the mouse over the target node and provide choice details.
        *   Use the "Add Choice..." button in the properties panel when a node is selected (only available if node doesn't have a "Next" link).
    *   **Next Node:** Use the "Set 'Next' Node..." button in the properties panel (only available if node has no choices).
    *   **Managing Connections:** Use the list in the properties panel. Double-click choice text to edit, double-click `[NEXT]` to convert to a choice, right-click for options (Remove, Edit, Focus, Clear Next). Click the target ID area (right side) to focus. Use "Clear All Connections" button to remove all outgoing links.
6.  **Start Node:**
    *   Right-click a node -> "Set as Start Node".
    *   The start node is often automatically renamed to `start` (configurable via `START_NODE_EXPORT_ID`) for export compatibility. You'll be prompted if a rename is needed for export.
7.  **Organizing:**
    *   Use **Bookmarks** (`View` menu or Node context menu) for quick access to key nodes.
    *   Use **Visual Groups** (Canvas context menu) to visually section your graph. Select groups to move them. Delete via context menu.
    *   Use **Path Highlighting** (Node context menu) to understand flow.
8.  **Saving/Loading/Import/Export:**
    *   Use the `File` menu to Save/Load `.dveditor` project files (preserves all editor state).
    *   Use `File -> Import/Export` for game-specific JSON formats (relies on `GAME_KEY_MAP` in `config.py`).
    *   Use `File -> Validate Dialogue...` before exporting.
9.  **Layout & Navigation:**
    *   Use `View -> Apply Auto-Layout` (`Ctrl+R`) to arrange nodes logically.
    *   Use `View -> Fit View to Content` (`Ctrl+F` shortcut from menu, *not* the Find shortcut) or zoom/pan manually.

## Configuration (`config.py`)

This file allows extensive customization:

*   **`ALLOWED_CUSTOM_PROPERTIES`:** Define data fields (name, type, default) specific to your needs that should appear in the properties panel.
*   **`PROJECT_KEY_MAP`:** Controls the JSON keys used when saving/loading the internal `.dveditor` project file (includes editor-specific keys like bookmarks/groups).
*   **`GAME_KEY_MAP`:** **Crucial for integration.** Defines how internal concepts (like `node_id`, `choices`, `next_node`, custom properties) map to the specific keys expected by your game's JSON dialogue format during import/export. Modify this carefully to match your target format.
*   **Colors & Dimensions:** Change the visual appearance of nodes, edges, highlights, groups, scene, etc.
*   **`NODE_COLORING_RULES`:** Define specific colors for nodes based on the character name field.
*   **Presets:** Define common choice types (e.g., "Accept", "Decline") with associated data (like icons or sounds) that can be added during export based on the preset selected for a choice.
*   **Validation Settings:** Enable/disable specific checks performed by the validator.
*   **Behavior:** Adjust debounce times, click thresholds, default start node ID for export, etc.

**Important:** Back up `config.py` before making significant changes.

## File Formats

*   **`.dveditor` (Project File):** A JSON file storing the complete state of the editor graph, including node data, positions, custom properties, connections, bookmarks, and visual groups. Designed for saving and resuming work within the editor. Structure defined by `PROJECT_KEY_MAP`.
*   **JSON (Import/Export):** A JSON file formatted according to a specific dialogue system requirements. The structure is defined by `GAME_KEY_MAP` in `config.py`. You **must** configure `GAME_KEY_MAP` correctly for import/export to work with your target system.

## Contributing

Please feel free to submit:

*   **Bug Reports:** Use the GitHub Issues tracker if applicable, or contact the author.
*   **Feature Requests:** Submit ideas via the Issues tracker or by contacting @wpenistone on Discord.

*Note: AI assistance was used in generating comments, documentation, and suggesting refactors/tests for this code.*

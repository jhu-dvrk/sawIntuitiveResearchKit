# TODO List

"Priority" issues to be completed for beta version are marked with a "⭐".

### General

- [x] Non-templated list model
- [x] Make edit/delete buttons styled as buttons
- [x] Hover/select list items
- [ ] Make sure config search paths are present
- [ ] Warning/message when loading config fails
- [ ] Disable delete button if not deletable
- [ ] Disable edit button if not editable, e.g. arms which can't have base frames
- [ ] Status/tool tips
- [ ] Make more keyboard friendly/better accessibility
- [ ] Move edit/delete buttons to base ItemView?
- [ ] Replace temporary in-line style sheets with custom QStyle
- [ ] Help buttons e.g. on dialogs
- [ ] Icons? e.g. delete, edit, add item
- [ ] Make prettier
- [ ] Do we want an overall create-config wizard?

### Config models

- [x] Initial I/O config model
- [x] Initial arm config model
- [x] Initial teleop config model
- [x] Initial console config model

- [x] (De)Serialization

- [ ] ⭐ Child components
- [ ] ⭐ Teleop disable rotation for ForceDimension
- [ ] ⭐ Arm name vs name_on_IO

- [ ] Full I/O config model (port address, etc.)
- [ ] Full arm config model
- [ ] Full teleop config model
- [ ] Full console config model

- [ ] Config values bounds
- [ ] Config validation

- [ ] Don't allow deleting parent items, e.g. `PSM1` if used in a `PSM1-MTMR` teleop
- [ ] Propagate renamings to children
- [ ] Don't allow duplicate ios/arms, multiple SUJ per i/o, MTM used twice in ECM teleop, etc.
- [ ] Warnings - missing SUJ
- [ ] Hide already chosen items from e.g. arm source list

### Config item views

- [x] Arm config item view
- [x] I/O config item view
- [x] Teleop config item view
- [x] Console config item view

- [ ] Confirm delete

### General editor

- [x] File name display
- [x] New config button
- [x] Open config
- [x] Save config
- [x] Dirty/clean save state marker
- [x] File dialogs install *.json filter
- [ ] Clean up file dialog state
- [ ] Intro view when no config is open
- [ ] Chatty/path/audio volume stuff
- [x] Mark unsaved when console input config changes

### Config item editors

- [x] I/O editor
- [x] Footpedal/head sensor I/O
- [ ] Focus controller I/O
- [x] Simple arm editor
- [x] Available native arm list in editor
- [x] Arm base frames
- [ ] Extended arm editor (derived/generic/socket types)
- [x] Teleop editor
- [ ] Derived teleop editor
- [x] Console editor
- [ ] ⭐ Put item name in editor title

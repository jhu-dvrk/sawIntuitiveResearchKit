# TODO List

### General

- [x] Non-templated list model
- [x] Make edit/delete buttons styled as buttons
- [x] Hover/select list items
- [ ] Disable delete button if not deletable
- [ ] Status/tool tips
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
- [ ] Initial console config model

- [ ] (De)Serialization

- [ ] Base frames
- [ ] Child components
- [ ] Teleop disable rotation

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
- [ ] Console config item view

- [ ] Confirm delete

### General editor

- [x] File name display
- [x] New config button
- [x] Open config
- [x] Save config
- [x] Dirty/clean save state marker
- [ ] File dialogs install *.json filter
- [ ] Clean up file dialog state
- [ ] Intro view when no config is open

### Config item editors

- [x] I/O editor
- [ ] Footpedal/head sensor I/O
- [x] Simple arm editor
- [x] Available native arm list in editor
- [ ] Arm base frames
- [ ] Extended arm editor (derived/generic/socket types)
- [x] Teleop editor
- [ ] Derived teleop editor
- [ ] Console editor

from PyQt5 import QtCore, QtGui, QtWidgets


HEADERS = [
    'Dataset',
    'Status',
    'Output',
    'ATE RMSE',
    'RPE 1s',
    'ATE Mean',
    'Aligned',
    'Overlap',
]

DARK_VIEWPORT = '#07101F'
DARK_ALT_ROW = '#0D1728'
DARK_HEADER = '#142033'
DARK_SELECTION = '#20375D'
DARK_GRID = '#223047'
TEXT_COLOR = '#F4F7FB'
MUTED_COLOR = '#A8B3C5'
REJECTED_BG = QtGui.QColor(48, 20, 26)
WARNING_BG = QtGui.QColor(47, 35, 15)
NORMAL_BG = QtGui.QColor(7, 16, 31)
ALT_BG = QtGui.QColor(13, 23, 40)


class ReadOnlyDelegate(QtWidgets.QStyledItemDelegate):
    """Strictly prevents editor widgets from being created."""

    def createEditor(self, parent, option, index):
        return None

    def editorEvent(self, event, model, option, index):
        # Keep checkbox/editor-style interactions disabled.
        return False


class DarkResultsTable(QtWidgets.QTableView):
    """
    Dark, read-only results table.

    This class exists because stylesheet-only fixes are not enough for the
    white viewport/editor artifact that can appear in QTableView on Windows.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('SummaryTable')
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.viewport().setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.viewport().setAutoFillBackground(True)
        self.setAutoFillBackground(True)
        self._apply_dark_palette()

    def _apply_dark_palette(self):
        palette = self.palette()
        palette.setColor(QtGui.QPalette.Base, QtGui.QColor(DARK_VIEWPORT))
        palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(DARK_ALT_ROW))
        palette.setColor(QtGui.QPalette.Window, QtGui.QColor(DARK_VIEWPORT))
        palette.setColor(QtGui.QPalette.Text, QtGui.QColor(TEXT_COLOR))
        palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(DARK_SELECTION))
        palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(TEXT_COLOR))
        self.setPalette(palette)

        viewport_palette = self.viewport().palette()
        viewport_palette.setColor(QtGui.QPalette.Base, QtGui.QColor(DARK_VIEWPORT))
        viewport_palette.setColor(QtGui.QPalette.Window, QtGui.QColor(DARK_VIEWPORT))
        viewport_palette.setColor(QtGui.QPalette.Text, QtGui.QColor(TEXT_COLOR))
        viewport_palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(DARK_SELECTION))
        viewport_palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(TEXT_COLOR))
        self.viewport().setPalette(viewport_palette)

    def mouseDoubleClickEvent(self, event):
        index = self.indexAt(event.pos())
        if index.isValid():
            self.selectRow(index.row())
        event.accept()

    def mousePressEvent(self, event):
        index = self.indexAt(event.pos())
        if index.isValid():
            self.selectRow(index.row())
        super().mousePressEvent(event)

    def keyPressEvent(self, event):
        # Block keys that may try to trigger edit behavior.
        if event.key() in (
            QtCore.Qt.Key_Return,
            QtCore.Qt.Key_Enter,
            QtCore.Qt.Key_F2,
        ):
            event.accept()
            return
        super().keyPressEvent(event)

    def paintEvent(self, event):
        # Fill the entire viewport before normal painting.
        # This prevents white empty areas on the right/bottom of the table.
        painter = QtGui.QPainter(self.viewport())
        painter.fillRect(self.viewport().rect(), QtGui.QColor(DARK_VIEWPORT))
        painter.end()
        super().paintEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.viewport().update()


class GlobalResultsModel(QtGui.QStandardItemModel):
    def __init__(self, parent=None):
        super().__init__(0, len(HEADERS), parent)
        self.setHorizontalHeaderLabels(HEADERS)
        self.rows = []

    def set_rows(self, rows, results_root, display_path, metric_text, colors):
        self.rows = list(rows)
        self.removeRows(0, self.rowCount())

        for row_index, row in enumerate(self.rows):
            output_full = row.get('estimate') or row.get('estimate_relpath') or '-'
            output_text = display_path(
                output_full,
                results_root if row.get('estimate') else None,
                max_chars=22,
            )

            values = [
                row.get('dataset', '-'),
                row.get('status', '-'),
                output_text,
                metric_text(row.get('ate_rmse_m')),
                metric_text(row.get('rpe_1s_rmse_m')),
                metric_text(row.get('ate_mean_m')),
                metric_text(row.get('aligned_samples'), 'count'),
                metric_text(row.get('overlap_duration_s'), 's'),
            ]

            row_background = self._background_for_row(row, row_index)
            foreground = QtGui.QBrush(QtGui.QColor(row.get('color', colors['text'])))

            items = []
            for column, value in enumerate(values):
                item = QtGui.QStandardItem(str(value))
                item.setEditable(False)
                item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
                item.setData(row, QtCore.Qt.UserRole)
                item.setForeground(foreground)
                item.setBackground(QtGui.QBrush(row_background))

                if column in (3, 4, 5, 6, 7):
                    item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                else:
                    item.setTextAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

                if column == 2:
                    item.setToolTip(str(output_full))
                else:
                    item.setToolTip(str(value))

                items.append(item)

            self.appendRow(items)

    @staticmethod
    def _background_for_row(row, row_index):
        status = row.get('status')
        if status in ('rejected', 'outlier', 'failed'):
            return REJECTED_BG

        if row.get('dataset') == 'MH_04_difficult' and row.get('ate_rmse_m') is not None:
            return WARNING_BG

        return ALT_BG if row_index % 2 else NORMAL_BG


def configure_results_table(table):
    table.setObjectName('SummaryTable')
    table.setItemDelegate(ReadOnlyDelegate(table))
    table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    table.setTextElideMode(QtCore.Qt.ElideRight)
    table.setWordWrap(False)

    table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
    table.setFocusPolicy(QtCore.Qt.NoFocus)

    table.setAlternatingRowColors(False)
    table.setShowGrid(True)
    table.setGridStyle(QtCore.Qt.SolidLine)

    table.verticalHeader().setVisible(False)
    table.setCornerButtonEnabled(False)

    table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)

    table.setSortingEnabled(False)
    table.setTabKeyNavigation(False)
    table.setDragEnabled(False)
    table.setAcceptDrops(False)
    table.setDropIndicatorShown(False)

    table.setStyleSheet(f"""
        QTableView#SummaryTable {{
            background: {DARK_VIEWPORT};
            background-color: {DARK_VIEWPORT};
            alternate-background-color: {DARK_ALT_ROW};
            color: {TEXT_COLOR};
            gridline-color: {DARK_GRID};
            border: 1px solid rgba(255, 255, 255, 0.08);
            border-radius: 9px;
            selection-background-color: {DARK_SELECTION};
            selection-color: {TEXT_COLOR};
            outline: 0;
        }}

        QTableView#SummaryTable::item {{
            color: {TEXT_COLOR};
            padding-left: 5px;
            padding-right: 5px;
            border: 0;
        }}

        QTableView#SummaryTable::item:selected {{
            background: {DARK_SELECTION};
            color: {TEXT_COLOR};
        }}

        QTableView#SummaryTable::viewport {{
            background: {DARK_VIEWPORT};
            background-color: {DARK_VIEWPORT};
        }}

        QHeaderView {{
            background: {DARK_HEADER};
            color: {MUTED_COLOR};
        }}

        QHeaderView::section {{
            background: {DARK_HEADER};
            color: {MUTED_COLOR};
            border: 0;
            border-right: 1px solid rgba(255, 255, 255, 0.05);
            padding: 6px;
            font-weight: 700;
        }}

        QTableCornerButton::section {{
            background: {DARK_HEADER};
            border: 0;
        }}

        QScrollBar:vertical {{
            background: {DARK_VIEWPORT};
            width: 10px;
            margin: 0;
            border: 0;
        }}

        QScrollBar::handle:vertical {{
            background: #26364F;
            border-radius: 5px;
            min-height: 20px;
        }}

        QScrollBar:horizontal {{
            background: {DARK_VIEWPORT};
            height: 10px;
            margin: 0;
            border: 0;
        }}

        QScrollBar::handle:horizontal {{
            background: #26364F;
            border-radius: 5px;
            min-width: 20px;
        }}

        QScrollBar::add-line,
        QScrollBar::sub-line {{
            width: 0;
            height: 0;
        }}

        QScrollBar::add-page,
        QScrollBar::sub-page {{
            background: {DARK_VIEWPORT};
        }}
    """)

    palette = table.palette()
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(DARK_VIEWPORT))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(DARK_ALT_ROW))
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(DARK_VIEWPORT))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor(TEXT_COLOR))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(DARK_SELECTION))
    palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(TEXT_COLOR))
    table.setPalette(palette)

    viewport = table.viewport()
    viewport.setAutoFillBackground(True)
    viewport_palette = viewport.palette()
    viewport_palette.setColor(QtGui.QPalette.Base, QtGui.QColor(DARK_VIEWPORT))
    viewport_palette.setColor(QtGui.QPalette.Window, QtGui.QColor(DARK_VIEWPORT))
    viewport.setPalette(viewport_palette)
    viewport.setStyleSheet(f'background-color: {DARK_VIEWPORT};')

    header = table.horizontalHeader()
    header.setStretchLastSection(False)
    header.setHighlightSections(False)
    header.setSectionsClickable(False)
    header.setDefaultAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

    fixed_columns = {
        0: 88,    # Dataset
        1: 72,    # Status
        2: 180,   # Output, elided with tooltip
        3: 74,    # ATE RMSE
        4: 66,    # RPE 1s
        5: 74,    # ATE Mean
        6: 58,    # Aligned
        7: 64,    # Overlap
    }

    for column, width in fixed_columns.items():
        header.setSectionResizeMode(column, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(column, width)

    header.setStretchLastSection(False)



    table.verticalHeader().setDefaultSectionSize(24)
    table.verticalHeader().setMinimumSectionSize(24)

    return table


def table_self_check(table):
    delegate = table.itemDelegate()
    viewport = table.viewport()
    base = viewport.palette().color(QtGui.QPalette.Base)
    window = viewport.palette().color(QtGui.QPalette.Window)

    dark_palette = base.lightness() < 96 and window.lightness() < 96
    dark_stylesheet = DARK_VIEWPORT.lower() in viewport.styleSheet().lower()

    model_ok = True
    model = table.model()
    if model is not None:
        for row in range(model.rowCount()):
            for column in range(model.columnCount()):
                item = model.item(row, column)
                if item is None:
                    continue
                if item.isEditable():
                    model_ok = False
                if item.flags() & QtCore.Qt.ItemIsEditable:
                    model_ok = False

    return {
        'table_no_edit': table.editTriggers() == QtWidgets.QAbstractItemView.NoEditTriggers and model_ok,
        'no_editor_delegate': isinstance(delegate, ReadOnlyDelegate)
        and delegate.createEditor(None, None, QtCore.QModelIndex()) is None,
        'row_selection': table.selectionBehavior() == QtWidgets.QAbstractItemView.SelectRows,
        'single_selection': table.selectionMode() == QtWidgets.QAbstractItemView.SingleSelection,
        'dark_results_table': isinstance(table, DarkResultsTable),
        'no_white_table_area': dark_palette or dark_stylesheet,
    }
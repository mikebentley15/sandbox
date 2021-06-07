from PyQt5.QtCore import (
    QAbstractItemModel,
    QFile,
    QIODevice,
    QModelIndex,
    Qt,
    QUrl
)
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtQuick import QQuickView

from TreeItem import TreeItem

class TreeModel(QAbstractItemModel):
    def __init__(self, data, parent=None):
        super().__init__(parent)
        
        self.rootItem = TreeItem(("Title", "Summary"))
        self.setupModelData(data.split('\n'), self.rootItem)
        
    def roleNames(self):
        roles = {
            Qt.UserRole + 1: b'TitleRole',
            Qt.UserRole + 2: b'SummaryRole',
            }
        return roles

    def columnCount(self, parent):
        if parent.isValid():
            return parent.internalPointer().columnCount()
        else:
            return self.rootItem.columnCount()

    def data(self, index, role):
        if not index.isValid():
            return None
        if role != Qt.DisplayRole:
            return None
        item = index.internalPointer()
        return item.data(index.column())

    def flags(self, index):
        if not index.isValid():
            return Qt.NoItemFlags
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable
    
    def headerData(self, section, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return self.rootItem.data(section)
        return None
    
    def index(self, row, column, parent):
        if not self.hasIndex(row, column, parent):
            return QModelIndex()
        
        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()
            
        childItem = parentItem.child(row)
        if childItem:
            return self.createIndex(row, column, childItem)
        else:
            return QModelIndex()
    
    def parent(self, index):
        if not index.isValid():
            return QModelIndex()
        childItem = index.internalPointer()
        parentItem = childItem.parent()
        if parentItem == self.rootItem:
            return QModelIndex()
        return self.createIndex(parentItem.row(), 0, parentItem)
    
    def rowCount(self, parent):
        if parent.column() > 0:
            return 0
        
        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()
        return parentItem.childCound()

    def setupModelData(self, lines, parent):
        parents = [parent]
        indentations = [0]
        
        number = 0
        
        while number < len(lines):
            position = 0
            while position < len(lines[number]):
                if lines[number][position] != ' ':
                    break
                position += 1
            lineData = lines[number][position:].trimmed()
            if lineData:
                # Read the column data from the rest of the line.
                columnData = [s for s in lineData.split('\t') if s]
                if position > indentations[-1]:
                    # The last child of the current parent is now the new
                    # parent unless the current parent has no children.
                    if parents[-1].childCount() > 0:
                        parents.append(parents[-1].child(parents[-1].childCount() - 1))
                        indentations.append(position)
            else:
                while position < indentations[-1] and len(parents) > 0:
                    parents.pop()
                    indentations.pop()
                continue
            parents[-1].appendChild(TreeItem(columnData, parents[-1]))
        number += 1


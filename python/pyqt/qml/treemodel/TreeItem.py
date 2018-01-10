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


class TreeItem(object):
    def __init__(self, data, parent=None):
        self.parentItem = parent
        self.itemData = data
        self.childItems = []

    def append_child(self, item):
        self.childItems.append(item)
    
    def child(self, row):
        return self.childItems[row]
    
    def childCount(self):
        return len(self.childItems)
    
    def columnCount(self):
        return len(self.itemData)
    
    def data(self, column):
        try:
            return self.itemData[column]
        except IndexError:
            return None
    
    def parent(self):
        return self.parentItem
    
    def row(self):
        if self.parentItem:
            return self.parentItem.childItems.index(self)
        return 0

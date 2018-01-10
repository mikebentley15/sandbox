import sys

from TreeModel import TreeModel

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

def main(arguments):
    app = QGuiApplication(sys.argv)
    view = QQuickView()
    f = QFile(':/default.txt')
    f.open(QIODevice.ReadOnly)
    model = TreeModel(f.readAll())
    f.close()
    
    rootContext = view.rootContext().setContextProperty('model', model)
    view.setSource(QUrl.fromLocalFile('TreeModel.qml'))
    view.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main(sys.argv)

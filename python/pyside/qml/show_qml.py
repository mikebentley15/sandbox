#!/usr/bin/env python3
'Run and show the given qml view file'

import sys
import argparse

from PySide2.QtWidgets import QApplication
from PySide2.QtQuick import QQuickView
from PySide2.QtCore import QUrl

def main(arguments):
    'Main logic here'
    parser = argparse.ArgumentParser(
            description='Run and show the given qml view file')
    parser.add_argument('qmlfile', help='qml view file')
    args = parser.parse_args(arguments)

    app = QApplication()
    view = QQuickView()
    url = QUrl(args.qmlfile)

    view.setSource(url)
    view.setResizeMode(QQuickView.SizeRootObjectToView)
    view.show()
    return app.exec_()

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

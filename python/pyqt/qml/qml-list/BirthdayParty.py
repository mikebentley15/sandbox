from PyQt5.QtCore import QObject, pyqtProperty
from PyQt5.QtQml import QQmlListProperty

from Person import Person

class BirthdayParty(QObject):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._guests = []

    @pyqtProperty(QQmlListProperty)
    def guests(self):
        return QQmlListProperty(Person, self, self._guests)

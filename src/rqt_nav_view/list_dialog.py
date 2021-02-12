# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Matthijs van der Burgh
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QDialog, QDialogButtonBox, QGroupBox, QHBoxLayout, QListWidget, QListWidgetItem, QPushButton, QVBoxLayout, QWidget


class ListDialog(QDialog):

    def __init__(self, title, items, parent):
        super(ListDialog, self).__init__(parent)

        self._layout = QVBoxLayout()
        self._view_box = QGroupBox("Optional topics")
        self._view_box_layout = QVBoxLayout(self._view_box)
        self._list_widget = QListWidget()
        self._view_box_layout.addWidget(self._list_widget)
        self._button_box = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Close)
        self._button_box.accepted.connect(self.save)
        self._button_box.rejected.connect(self.reject)

        self.setWindowTitle(title)

        self.create_list_widget(items)

        self._layout.addWidget(self._view_box)
        self._layout.addWidget(self._button_box)
        self.setLayout(self._layout)

        # createConnections
        self._list_widget.itemChanged.connect(self.highlight_checked)

        self.selected_items = []

    def create_list_widget(self, items):
        for item in items:
            w_item = QListWidgetItem()
            w_item.setText(item[0])
            w_item.setFlags(w_item.flags() | Qt.ItemIsUserCheckable)
            w_item.setCheckState(Qt.Checked if item[1] else Qt.Unchecked)
            self._list_widget.addItem(w_item)
        self._list_widget.setMinimumWidth(self._list_widget.sizeHintForColumn(0) + 5)

    @staticmethod
    def highlight_checked(item):
        if item.checkState() == Qt.Checked:
            item.setBackground(QColor("#ffffb2"))
        else:
            item.setBackground(QColor("#ffffff"))

    def save(self):
        self.selected_items = []
        for i in range(len(self._list_widget)):
            item = self._list_widget.item(i)
            if item.checkState() == Qt.Checked:
                self.selected_items.append(item.text())

        self.accept()

    def exec_(self):
        error_code = super(ListDialog, self).exec_()
        return self.selected_items, error_code

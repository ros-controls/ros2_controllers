#!/usr/bin/env python

# Copyright 2022 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QWidget


class DoubleEditor(QWidget):
    # TODO:
    # - Actually make bounds optional
    #
    # - Support wrapping mode
    #
    # - Support unspecified (+-Inf) lower and upper bounds (both, or one)
    #
    # - Allow to specify the step and page increment sizes
    #   (right-click context menu?)
    #
    # - Use alternative widget to slider for values that wrap, or are
    #   unbounded.
    #   QwtWheel could be a good choice, dials are not so good because they
    #   use lots of vertical (premium) screen space, and are fine for wrapping
    #   values, but not so much for unbounded ones
    #
    # - Merge with existing similar code such as rqt_reconfigure's
    #   DoubleEditor?
    """
    Widget that allows to edit the value of a floating-point value.

    Optionally subject to lower and upper bounds.
    """

    valueChanged = Signal(float)

    def __init__(self, min_val, max_val):
        super().__init__()

        # Preconditions
        assert min_val < max_val

        # Cache values
        self._min_val = min_val
        self._max_val = max_val

        # Load editor UI
        ui_file = os.path.join(
            get_package_share_directory("rqt_joint_trajectory_controller"),
            "resource",
            "double_editor.ui",
        )
        loadUi(ui_file, self)

        # Setup widget ranges and slider scale factor
        self.slider.setRange(0, 100)
        self.slider.setSingleStep(1)
        self._scale = (max_val - min_val) / (self.slider.maximum() - self.slider.minimum())
        self.spin_box.setRange(min_val, max_val)
        self.spin_box.setSingleStep(self._scale)

        # Couple slider and spin box together
        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spin_box.valueChanged.connect(self._on_spinbox_changed)

        # Ensure initial sync of slider and spin box
        self._on_spinbox_changed()

    def _slider_to_val(self, sval):
        return self._min_val + self._scale * (sval - self.slider.minimum())

    def _val_to_slider(self, val):
        return round(self.slider.minimum() + (val - self._min_val) / self._scale)

    def _on_slider_changed(self):
        val = self._slider_to_val(self.slider.value())
        self.spin_box.blockSignals(True)  # Prevents updating the command twice
        self.spin_box.setValue(val)
        self.spin_box.blockSignals(False)
        self.valueChanged.emit(val)

    def _on_spinbox_changed(self):
        val = self.spin_box.value()
        self.slider.blockSignals(True)  # Prevents updating the command twice
        self.slider.setValue(self._val_to_slider(val))
        self.slider.blockSignals(False)
        self.valueChanged.emit(val)

    def setValue(self, val):
        if val != self.spin_box.value():
            self.spin_box.blockSignals(True)
            self.spin_box.setValue(val)  # Update spin box first
            self._on_spinbox_changed()  # Sync slider with spin box
            self.spin_box.blockSignals(False)

    def value(self):
        return self.spin_box.value()

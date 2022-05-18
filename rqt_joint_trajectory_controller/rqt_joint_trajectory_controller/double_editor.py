#!/usr/bin/env python

# Copyright (C) 2014, PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of PAL Robotics S.L. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
        super(DoubleEditor, self).__init__()

        # Preconditions
        assert min_val < max_val

        # Cache values
        self._min_val = min_val
        self._max_val = max_val

        # Load editor UI
        ui_file = os.path.join(get_package_share_directory('rqt_joint_trajectory_controller'),
                               'resource', 'double_editor.ui')
        loadUi(ui_file, self)

        # Setup widget ranges and slider scale factor
        self.slider.setRange(0, 100)
        self.slider.setSingleStep(1)
        self._scale = (max_val - min_val) / \
                      (self.slider.maximum() - self.slider.minimum())
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
        return round(self.slider.minimum() + (val - self._min_val) /
                     self._scale)

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

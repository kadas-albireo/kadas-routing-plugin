import os
import logging

from qgis.PyQt import uic
from qgis.PyQt.QtGui import QIcon
from qgis.PyQt.QtWidgets import QWidget

from kadasrouting.gui.locationinputwidget import LocationInputWidget
from kadasrouting.utilities import iconPath

from qgis.core import (
    QgsCoordinateReferenceSystem,
    QgsWkbTypes,
    QgsGeometry,
    QgsMarkerSymbol,
    QgsSvgMarkerSymbolLayer,
)
from qgis.gui import QgsRubberBand
from qgis.utils import iface

from kadasrouting.gui.valhallaroutebottombar import ValhallaRouteBottomBar

WIDGET, BASE = uic.loadUiType(
    os.path.join(os.path.dirname(__file__), "optimalroutebottombar.ui")
)

LOG = logging.getLogger(__name__)


class OptimalRouteBottomBar(ValhallaRouteBottomBar, WIDGET):
    def __init__(self, canvas, action, plugin):
        self.default_layer_name = "Route"
        self.canvas = canvas
        super().__init__(canvas, action, plugin)
        self.btnAddWaypoints.setIcon(QIcon(":/kadas/icons/add"))
        self.btnAddWaypoints.setToolTip(self.tr("Add waypoint"))
        self.waypointsSearchBox = LocationInputWidget(
            self.canvas, locationSymbolPath=iconPath("pin_bluegray.svg")
        )
        self.groupBox.layout().addWidget(self.waypointsSearchBox, 0, 0)
        self.btnAddWaypoints.clicked.connect(self.addWaypoints)
        self.btnAreasToAvoidFromCanvas.toggled.connect(self.setPolygonDrawingMapTool)
            
    def clearPoints(self):
        self.waypointsSearchBox.clearSearchBox()
        self.waypoints = []
        self.lineEditWaypoints.clear()
        self.waypointPins = []
        super().clearPoints()

    def addWaypoints(self):
        """Add way point to the list of way points"""
        if self.waypointsSearchBox.text() == "":
            return
        waypoint = self.waypointsSearchBox.point
        self.waypoints.append(waypoint)
        if self.lineEditWaypoints.text() == "":
            self.lineEditWaypoints.setText(self.waypointsSearchBox.text())
        else:
            self.lineEditWaypoints.setText(
                self.lineEditWaypoints.text() + ";" + self.waypointsSearchBox.text()
            )
        self.waypointsSearchBox.clearSearchBox()
        # Remove way point pin from the location input widget
        self.waypointsSearchBox.removePin()
        # Create/add new waypoint pin for the waypoint
        self.addWaypointPin(waypoint)

    def reverse(self):
        super().reverse()
        # Reverse waypoints' order
        self.waypoints.reverse()
        self.waypointPins.reverse()
        # Reverse the text on the line edit
        self.lineEditWaypoints.setText(
            ";".join(reversed(self.lineEditWaypoints.text().split(";")))
        )

    def addWaypointPin(self, waypoint):
        """Create a new pin for a waypoint with its symbology"""
        # Create pin with waypoint symbology
        waypointPin = QgsRubberBand(self.canvas, QgsWkbTypes.PointGeometry)
        waypointPin.setToGeometry(QgsGeometry.fromPointXY(waypoint), QgsCoordinateReferenceSystem("EPSG:4326"))


        symbol = QgsMarkerSymbol()
        symbol.deleteSymbolLayer(0)

        svg_layer = QgsSvgMarkerSymbolLayer(":/kadas/icons/waypoint")
        # Anchor: center horizontally, bottom vertically
        svg_layer.setHorizontalAnchorPoint(QgsSvgMarkerSymbolLayer.HorizontalAnchorPoint.Center)
        svg_layer.setVerticalAnchorPoint(QgsSvgMarkerSymbolLayer.VerticalAnchorPoint.Center)
        svg_layer.setSize(16)
        symbol.appendSymbolLayer(svg_layer)
        waypointPin.setSymbol(symbol)

        self.waypointPins.append(waypointPin)
        self.canvas.refresh()

    def clearPins(self):
        """Remove all pins from the map
        Not removing the point stored.
        """
        # remove waypoint pins
        self.waypointPins = []
        super().clearPins()
        self.canvas.refresh()

    def addPins(self):
        """Add pins for all stored points."""
        for waypoint in self.waypoints:
            self.addWaypointPin(waypoint)
        super().addPins()

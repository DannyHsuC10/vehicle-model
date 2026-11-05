"""Construction-line rendering checks without opening a native Tk window."""
from io import BytesIO
import unittest
from unittest.mock import Mock

from suspension_geometry_lab.chart_base import Chart


class ChartOverlayTests(unittest.TestCase):
    def chart(self):
        chart=object.__new__(Chart)
        chart.canvas=Mock()
        chart.canvas.winfo_width.return_value=800
        chart.canvas.winfo_height.return_value=470
        chart.spec=None;chart.view=None;chart.primitives=[]
        chart.azimuth=-.65;chart.elevation=.5
        return chart

    def test_distant_construction_lines_are_clipped_without_changing_fit(self):
        chart=self.chart()
        physical=[([0,-1,0],[0,1,1],"upper")]
        def spec(end):
            return {"projection":"Front","segments":physical,
                    "overlays":[([0,0,.5],[0,end,.5],"projection_upper")]}
        chart.set(spec(1));near=list(chart.view)
        chart.set(spec(1e8))
        self.assertEqual(chart.view,near)
        left,top,right,bottom=chart.bounds
        dashed=[p for p in chart.primitives if p[0]=="line" and p[4]]
        self.assertTrue(dashed)
        # The first dashed primitive is the overlay; the last is its legend.
        x1,y1,x2,y2=dashed[0][1]
        self.assertTrue(left<=x1<=right and left<=x2<=right)
        self.assertTrue(top<=y1<=bottom and top<=y2<=bottom)

    def test_svg_and_png_preserve_dashed_line_patterns(self):
        chart=self.chart()
        chart.line([10,20,100,20],"#fb7185",2,(7,4))
        self.assertIn('stroke-dasharray="7 4"',chart.svg())
        try:from PIL import Image
        except ImportError:self.skipTest("Pillow PNG export is optional")
        stream=BytesIO();stream.name="dashes.png"
        chart.png(stream)
        with Image.open(stream) as image:
            self.assertEqual(image.getpixel((24,40)),(251,113,133))
            self.assertEqual(image.getpixel((38,40)),(15,23,42))

    def test_geometry_legends_wrap_and_keep_overlay_keys(self):
        chart=self.chart();chart.canvas.winfo_width.return_value=600
        chart.set({"projection":"Side","segments":[([0,0,0],[1,0,1],kind)
                   for kind in ("upper","lower","upright","rod","wheel","chassis","ground")],
                   "overlays":[([0,0,0],[2,0,2],kind) for kind in
                               ("projection_upper","projection_lower","contact_line","kingpin")]})
        labels={p[4]:p[1] for p in chart.primitives if p[0]=="text"}
        for label in ("Upper projection","Lower projection","Contact to IC","Kingpin"):
            self.assertIn(label,labels)
            self.assertLess(labels[label][1],chart.bounds[1])
        self.assertGreater(chart.bounds[1],64)

    def test_toggling_layers_preserves_camera_bounds_and_equal_metric_scale(self):
        chart=self.chart()
        chart.canvas.winfo_width.return_value=600
        chart.set({"projection":"Front","segments":[([0,-1,0],[0,1,1],"upper")]})
        original_view=list(chart.view);original_bounds=chart.bounds
        for overlays in ([([0,0,0],[0,100,.5],kind) for kind in
                          ("projection_upper","projection_lower","contact_line","kingpin")],[]):
            chart.spec["overlays"]=overlays
            chart.draw()  # GUI display toggles retain the existing camera.
            self.assertEqual(chart.view,original_view)
            self.assertEqual(chart.bounds,original_bounds)
            left,top,right,bottom=chart.bounds
            xmin,xmax,ymin,ymax=chart.view
            self.assertAlmostEqual((right-left)/(xmax-xmin),(bottom-top)/(ymax-ymin))

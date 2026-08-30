import unittest

from PySide6.QtWidgets import QApplication, QLabel

from swphysics.gui import (
    APP_AUTHOR,
    APP_NAME,
    OptimizerWindow,
    configure_application_metadata,
)


class GuiMetadataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])
        configure_application_metadata(cls.application)

    def test_qt_application_metadata_uses_the_requested_author(self):
        self.assertEqual(APP_NAME, self.application.applicationName())
        self.assertEqual(APP_AUTHOR, self.application.organizationName())

    def test_author_is_visible_in_the_main_window(self):
        window = OptimizerWindow()
        self.addCleanup(window.close)
        labels = tuple(label.text() for label in window.findChildren(QLabel))
        self.assertIn("Author: IrisNuiYaMa_164", labels)


if __name__ == "__main__":
    unittest.main()

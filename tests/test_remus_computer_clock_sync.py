import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = ROOT / "platforms" / "esp32" / "src" / "app" / "RemusApp.cpp"


class RemusComputerClockSyncSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_computer_gatt_server_exposes_clock_sync_characteristic(self):
        self.assertIn("pClockSyncCharacteristic = pService->createCharacteristic(", self.source)
        self.assertIn("BLADE_CLOCK_SYNC_CHARACTERISTIC_UUID", self.source)
        self.assertIn("BLECharacteristic::PROPERTY_WRITE", self.source)
        self.assertIn("BLECharacteristic::PROPERTY_NOTIFY", self.source)
        self.assertIn("setCallbacks(new RemusClockSyncCallbacks())", self.source)

    def test_computer_response_carries_its_own_boot_identity(self):
        self.assertIn("protocol::encodeClockSyncResponse(", self.source)
        self.assertIn("computerBootId);", self.source)


if __name__ == "__main__":
    unittest.main()

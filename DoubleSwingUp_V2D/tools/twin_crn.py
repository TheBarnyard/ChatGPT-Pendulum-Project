"""run_job with the Arduino USB delay drawn from its own random stream, so the radio, encoder noise and
receiver timing are the same random sequence whatever the script writes to the Arduino (paired A/B)."""
import os, sys, random
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import twin_rig
twin_rig.TW = os.environ.get("TWINPY", "twinpy")
ARD_BYTE = 10.0 / 115200.0
def run_job(job):
    sys.path.insert(0, twin_rig.TW)
    import twin_hw
    def ard_write(self, data):
        if not hasattr(self, "_usb_rng"):
            self._usb_rng = random.Random(int(self.cfg["run"]["seed"]) * 7919 + 1)
        u = self.cfg["usb"]
        arrive = self.t + self._usb_rng.uniform(0.1, max(0.1, float(u["out_ms"]))) / 1000.0 + len(data) * ARD_BYTE
        self.at(arrive, self._ard_bytes, (self.ard_epoch, bytes(data)))
    twin_hw.Twin.ard_write = ard_write
    return twin_rig.run_job(job)

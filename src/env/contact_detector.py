from sbr_pjt.msg import Contacts

class ContactDetector():
    def __init__(self, env):
        self.env = env
        self.subscription = self.env.create_subscription(Contacts, '/contacts', self._callback, 10)
        self.game_over = False
        self.enable_detection = False

    def _callback(self, msg):
        for contact in msg.states:
            if "base_link" in contact.info and "ground_plane" in contact.info:
                if self.enable_detection:
                    self.game_over = True
                    break

    def has_fallen(self):
        return self.game_over

    def set_enable_detection(self, enable: bool):
        self.enable_detection = enable

    def reset(self):
        self.game_over = False
        self.enable_detection = False

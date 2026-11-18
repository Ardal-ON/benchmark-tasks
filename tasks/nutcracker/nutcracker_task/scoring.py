"""Visible compression, custody and irreversible crack evidence; no seam input."""
from dataclasses import dataclass

@dataclass
class Rubric:
    witness_valid: bool = False
    witness_poisoned: bool = False
    fracture_qualified: bool = False
    opening_seen: bool = False
    visible_crack_qualified: bool = False
    full_width_seen: bool = False
    crack_hold: int = 0
    seat_hold: int = 0

    def observe(self, *, opening_m, opposing_jaw_contact, robot_shell_contact,
                compression=False, mouth_custody=True, foreign_contact=False):
        foreign = robot_shell_contact or foreign_contact
        if self.witness_valid and (foreign or not mouth_custody):
            self.witness_valid = False
            self.witness_poisoned = True
        # Evidence visible in this sample must have a witness from an EARLIER
        # sample. Same-sample compression cannot retroactively qualify a crack.
        measurable = opening_m >= 0.0005
        opened = opening_m >= 0.003
        if measurable and not self.opening_seen:
            self.fracture_qualified = bool(self.witness_valid and not foreign and mouth_custody)
        self.opening_seen = self.opening_seen or measurable
        if opened and not self.full_width_seen:
            self.visible_crack_qualified = bool(self.fracture_qualified and
                self.witness_valid and not foreign and mouth_custody)
        self.full_width_seen = self.full_width_seen or opened
        if (compression and opposing_jaw_contact and mouth_custody and
                not foreign and not self.witness_poisoned and not self.opening_seen):
            self.witness_valid = True

    def update(self, *, opening_m, opposing_jaw_contact, robot_shell_contact,
               seated, fixture_stable, contained, clear, compression=False,
               mouth_custody=True, foreign_contact=False, observe_evidence=True):
        if observe_evidence:
            self.observe(opening_m=opening_m,opposing_jaw_contact=opposing_jaw_contact,
                robot_shell_contact=robot_shell_contact,compression=compression,
                mouth_custody=mouth_custody,foreign_contact=foreign_contact)
        # A correctly documented crack can close after unloading. The final
        # containment/release hold still must last a full uninterrupted 2 s.
        safe = fixture_stable and contained and clear
        self.crack_hold = self.crack_hold + 1 if self.visible_crack_qualified and safe else 0
        self.seat_hold = self.seat_hold + 1 if not self.opening_seen and seated and safe else 0
        return 1.0 if self.crack_hold >= 40 else 0.25 if self.seat_hold >= 40 else 0.0

"""Species-name resolution and policy generation without model tooling."""
import unittest

from tools.species_profile import (
    DEFAULT_MODELS, HUMAN_VOCAL, WORLDWIDE, load_labels, read_esp_profile, resolve_species,
    render_profile,
)


class SpeciesProfileTest(unittest.TestCase):
    def test_international_selection_accepts_names_in_classifier_order(self):
        labels = load_labels((WORLDWIDE / "labels.txt").read_bytes())
        selected = resolve_species(
            ["# Comments are ignored", "bubo virginianus", "", " North Island Brown  Kiwi ",
             "Bubo virginianus_Great Horned Owl"], labels,
        )
        self.assertEqual([labels[i] for i in selected], [
            "Apteryx mantelli_North Island Brown Kiwi", "Bubo virginianus_Great Horned Owl",
        ])

    def test_unknown_and_ambiguous_names_report_the_input_lines(self):
        labels = ["A one_Bird", "B two_Bird"]
        with self.assertRaisesRegex(ValueError, r"line 1: .*ambiguous[\s\S]*line 2: .*absent"):
            resolve_species(["Bird", "Not present"], labels)

    def test_exact_label_can_disambiguate_a_common_name(self):
        self.assertEqual(resolve_species(["b two_bird"], ["A one_Bird", "B two_Bird"]), [1])

    def test_empty_list_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "at least one species"):
            resolve_species(["", " # comment"], ["A one_Bird"])

    def test_guard_stays_available_without_becoming_a_detection(self):
        header = render_profile(["A one_Bird", HUMAN_VOCAL], True, "test")
        self.assertIn("kCount = 2", header)
        self.assertIn("kHumanVocal = 1", header)
        self.assertIn("kVocalGuardOnly = true", header)

    def test_default_model_keeps_all_1018_visible_classes(self):
        header = read_esp_profile(DEFAULT_MODELS)
        self.assertIn("kCount = 1018", header)
        self.assertIn("kHumanVocal = 460", header)
        self.assertIn("kVocalGuardOnly = false", header)


if __name__ == "__main__":
    unittest.main()

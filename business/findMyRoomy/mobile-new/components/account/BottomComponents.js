import React from "react";
import {
  View,
  Text,
  StyleSheet,
  ScrollView,
  TouchableOpacity,
  TextInput,
  Image,
  Alert,
  Animated,
  Dimensions,
  Platform,
} from "react-native";
import { Ionicons } from "@expo/vector-icons";
import * as Haptics from "expo-haptics";

const { width } = Dimensions.get("window");

/* ===================== BASICS ===================== */
export const Basics = ({
  editing,
  profile,
  setProfile,
  toggleEdit,
  QUIZ,
  SingleSelectEditor,
  getOptionLabel,
  formatBudgetRange,
  formatDuration,
}) => (
  <View style={styles.modernSection}>
    <View style={styles.modernSectionHeader}>
      <View style={styles.sectionTitleContainer}>
        <Ionicons name="home-outline" size={20} color="#EF4444" />
        <Text style={styles.modernSectionTitle}>Housing Basics</Text>
      </View>
      <TouchableOpacity
        style={styles.editChip}
        onPress={() => toggleEdit("basics")}
        activeOpacity={0.8}
      >
        <Text style={styles.editChipText}>
          {editing.basics ? "Done" : "Edit"}
        </Text>
      </TouchableOpacity>
    </View>

    {editing.basics ? (
      <ScrollView
        style={styles.editingContainer}
        showsVerticalScrollIndicator={false}
      >
        {/* Budget */}
        <View style={styles.budgetEditorContainer}>
          <Text style={styles.editorSectionTitle}>Budget Range</Text>
          <View style={styles.budgetInputsContainer}>
            <View style={styles.budgetInputWrapper}>
              <Text style={styles.budgetInputLabel}>Min</Text>
              <TextInput
                style={styles.budgetInput}
                value={String(profile.basics.budget_range[0])}
                onChangeText={(t) => {
                  const val = parseInt(t, 10) || 0;
                  setProfile((p) => ({
                    ...p,
                    basics: {
                      ...p.basics,
                      budget_range: [val, p.basics.budget_range[1]],
                    },
                  }));
                }}
                keyboardType="numeric"
                placeholder="900"
              />
            </View>
            <Text style={styles.budgetSeparator}>to</Text>
            <View style={styles.budgetInputWrapper}>
              <Text style={styles.budgetInputLabel}>Max</Text>
              <TextInput
                style={styles.budgetInput}
                value={String(profile.basics.budget_range[1])}
                onChangeText={(t) => {
                  const val = parseInt(t, 10) || 0;
                  setProfile((p) => ({
                    ...p,
                    basics: {
                      ...p.basics,
                      budget_range: [p.basics.budget_range[0], val],
                    },
                  }));
                }}
                keyboardType="numeric"
                placeholder="1400"
              />
            </View>
          </View>
        </View>

        <SingleSelectEditor
          title="When are you looking to move?"
          options={QUIZ.sections.basics.questions.move_in_timeline.optionsOrder}
          selected={profile.basics.move_in_timeline}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              basics: { ...p.basics, move_in_timeline: value },
            }))
          }
          sectionKey="basics"
          questionKey="move_in_timeline"
        />

        <SingleSelectEditor
          title="Room type preference"
          options={QUIZ.sections.basics.questions.room_type.optionsOrder}
          selected={profile.basics.room_type}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              basics: { ...p.basics, room_type: value },
            }))
          }
          sectionKey="basics"
          questionKey="room_type"
        />

        <SingleSelectEditor
          title="Preferred lease duration"
          options={QUIZ.sections.basics.questions.lease_duration.optionsOrder}
          selected={profile.basics.lease_duration}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              basics: { ...p.basics, lease_duration: value },
            }))
          }
          sectionKey="basics"
          questionKey="lease_duration"
        />

        <View style={styles.durationPickerContainer}>
          <Text style={styles.editorSectionTitle}>Minimum Stay</Text>
          <View style={styles.durationInputsContainer}>
            <TextInput
              style={styles.durationInput}
              value={String(profile.basics.minimum_stay ?? "")}
              onChangeText={(t) => {
                const months = parseInt(t, 10) || 1;
                setProfile((p) => ({
                  ...p,
                  basics: { ...p.basics, minimum_stay: months },
                }));
              }}
              keyboardType="numeric"
              placeholder="12"
            />
            <Text style={styles.durationLabel}>months</Text>
          </View>
        </View>

        <View style={styles.durationPickerContainer}>
          <Text style={styles.editorSectionTitle}>Maximum Stay</Text>
          <View style={styles.durationInputsContainer}>
            <TextInput
              style={styles.durationInput}
              value={
                profile.basics.maximum_stay
                  ? String(profile.basics.maximum_stay)
                  : ""
              }
              onChangeText={(t) => {
                const months = t ? parseInt(t, 10) || null : null;
                setProfile((p) => ({
                  ...p,
                  basics: { ...p.basics, maximum_stay: months },
                }));
              }}
              keyboardType="numeric"
              placeholder="Open-ended"
            />
            <Text style={styles.durationLabel}>
              months (leave blank for open-ended)
            </Text>
          </View>
        </View>
      </ScrollView>
    ) : (
      <View style={styles.detailsContainer}>
        <View style={styles.detailCard}>
          <Text style={styles.detailCardTitle}>Budget & Timeline</Text>
          <View style={styles.modernDetailRow}>
            <View style={styles.modernDetailLeft}>
              <Ionicons name="wallet-outline" size={16} color="#6B7280" />
              <Text style={styles.modernDetailLabel}>Budget</Text>
            </View>
            <Text style={styles.modernDetailValue}>
              {formatBudgetRange(profile.basics.budget_range)}
            </Text>
          </View>

          <View style={styles.modernDetailRow}>
            <View style={styles.modernDetailLeft}>
              <Ionicons name="time-outline" size={16} color="#6B7280" />
              <Text style={styles.modernDetailLabel}>Min Stay</Text>
            </View>
            <Text style={styles.modernDetailValue}>
              {formatDuration(profile.basics.minimum_stay ?? undefined)}
            </Text>
          </View>

          <View style={styles.modernDetailRow}>
            <View style={styles.modernDetailLeft}>
              <Ionicons name="calendar-outline" size={16} color="#6B7280" />
              <Text style={styles.modernDetailLabel}>Room Type</Text>
            </View>
            <Text style={styles.modernDetailValue}>
              {getOptionLabel("basics", "room_type", profile.basics.room_type)}
            </Text>
          </View>

          <View style={styles.modernDetailRow}>
            <View style={styles.modernDetailLeft}>
              <Ionicons
                name="document-text-outline"
                size={16}
                color="#6B7280"
              />
              <Text style={styles.modernDetailLabel}>Lease Duration</Text>
            </View>
            <Text style={styles.modernDetailValue}>
              {getOptionLabel(
                "basics",
                "lease_duration",
                profile.basics.lease_duration
              )}
            </Text>
          </View>
        </View>
      </View>
    )}
  </View>
);

/* ===================== LIFESTYLE ===================== */
export const Lifestyle = ({
  editing,
  profile,
  setProfile,
  toggleEdit,
  QUIZ,
  SingleSelectEditor,
  ScaleEditor,
  getOptionLabel,
}) => (
  <View style={styles.modernSection}>
    <View style={styles.modernSectionHeader}>
      <View style={styles.sectionTitleContainer}>
        <Ionicons name="leaf-outline" size={20} color="#8B5CF6" />
        <Text style={styles.modernSectionTitle}>Lifestyle</Text>
      </View>
      <TouchableOpacity
        style={styles.editChip}
        onPress={() => toggleEdit("lifestyle")}
        activeOpacity={0.8}
      >
        <Text style={styles.editChipText}>
          {editing.lifestyle ? "Done" : "Edit"}
        </Text>
      </TouchableOpacity>
    </View>

    {editing.lifestyle ? (
      <ScrollView
        style={styles.editingContainer}
        showsVerticalScrollIndicator={false}
      >
        <ScaleEditor
          title="Cleanliness standard (shared spaces)"
          value={profile.lifestyle.cleanliness_level}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              lifestyle: { ...p.lifestyle, cleanliness_level: value },
            }))
          }
          leftAnchor="Lived-in"
          rightAnchor="Spotless"
        />

        <ScaleEditor
          title="How often you clean shared areas"
          value={profile.lifestyle.cleaning_frequency}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              lifestyle: { ...p.lifestyle, cleaning_frequency: value },
            }))
          }
          leftAnchor="Monthly"
          rightAnchor="Daily"
        />

        <SingleSelectEditor
          title="You're typically…"
          options={QUIZ.sections.lifestyle.questions.home_presence.optionsOrder}
          selected={profile.lifestyle.home_presence}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              lifestyle: { ...p.lifestyle, home_presence: value },
            }))
          }
          sectionKey="lifestyle"
          questionKey="home_presence"
        />

        <SingleSelectEditor
          title="Daily rhythm"
          options={QUIZ.sections.lifestyle.questions.sleep_rhythm.optionsOrder}
          selected={profile.lifestyle.sleep_rhythm}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              lifestyle: { ...p.lifestyle, sleep_rhythm: value },
            }))
          }
          sectionKey="lifestyle"
          questionKey="sleep_rhythm"
        />

        <SingleSelectEditor
          title="Friends over (per week)"
          options={
            QUIZ.sections.lifestyle.questions.guest_frequency.optionsOrder
          }
          selected={profile.lifestyle.guest_frequency}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              lifestyle: { ...p.lifestyle, guest_frequency: value },
            }))
          }
          sectionKey="lifestyle"
          questionKey="guest_frequency"
        />

        <SingleSelectEditor
          title="Overnight guests"
          options={
            QUIZ.sections.lifestyle.questions.overnight_guests.optionsOrder
          }
          selected={profile.lifestyle.overnight_guests}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              lifestyle: { ...p.lifestyle, overnight_guests: value },
            }))
          }
          sectionKey="lifestyle"
          questionKey="overnight_guests"
        />

        <ScaleEditor
          title="Noise tolerance at home"
          value={profile.lifestyle.noise_tolerance}
          onChange={(value) =>
            setProfile((p) => ({
              ...p,
              lifestyle: { ...p.lifestyle, noise_tolerance: value },
            }))
          }
          leftAnchor="Very quiet"
          rightAnchor="Don't mind activity"
        />
      </ScrollView>
    ) : (
      <View style={styles.lifestyleGrid}>
        <View style={styles.lifestyleItem}>
          <Text style={styles.lifestyleLabel}>Cleanliness</Text>
          <View style={styles.scaleContainer}>
            {[1, 2, 3, 4, 5].map((level) => (
              <View
                key={level}
                style={[
                  styles.scaleDot,
                  profile.lifestyle.cleanliness_level >= level &&
                    styles.scaleDotActive,
                ]}
              />
            ))}
          </View>
          <Text style={styles.scaleValue}>
            {profile.lifestyle.cleanliness_level}/5
          </Text>
        </View>

        <View style={styles.lifestyleItem}>
          <Text style={styles.lifestyleLabel}>Sleep Schedule</Text>
          <Text style={styles.lifestyleValue}>
            {getOptionLabel(
              "lifestyle",
              "sleep_rhythm",
              profile.lifestyle.sleep_rhythm
            )}
          </Text>
        </View>

        <View style={styles.lifestyleItem}>
          <Text style={styles.lifestyleLabel}>Home Presence</Text>
          <Text style={styles.lifestyleValue}>
            {getOptionLabel(
              "lifestyle",
              "home_presence",
              profile.lifestyle.home_presence
            )}
          </Text>
        </View>

        <View style={styles.lifestyleItem}>
          <Text style={styles.lifestyleLabel}>Noise Tolerance</Text>
          <View style={styles.scaleContainer}>
            {[1, 2, 3, 4, 5].map((level) => (
              <View
                key={level}
                style={[
                  styles.scaleDot,
                  profile.lifestyle.noise_tolerance >= level &&
                    styles.scaleDotActive,
                ]}
              />
            ))}
          </View>
          <Text style={styles.scaleValue}>
            {profile.lifestyle.noise_tolerance}/5
          </Text>
        </View>
      </View>
    )}
  </View>
);



/* ===================== PETS ===================== */
export const Pets = ({
  editing,
  profile,
  setProfile,
  toggleEdit,
  QUIZ,
  MultiSelectEditor,
  getOptionLabel,
}) => (
  <View style={styles.modernSection}>
    <View style={styles.modernSectionHeader}>
      <View style={styles.sectionTitleContainer}>
        <Ionicons name="paw-outline" size={20} color="#F97316" />
        <Text style={styles.modernSectionTitle}>Pets</Text>
      </View>
      <TouchableOpacity
        style={styles.editChip}
        onPress={() => toggleEdit("pets")}
        activeOpacity={0.8}
      >
        <Text style={styles.editChipText}>
          {editing.pets ? "Done" : "Edit"}
        </Text>
      </TouchableOpacity>
    </View>

    {editing.pets ? (
      <ScrollView
        style={styles.editingContainer}
        showsVerticalScrollIndicator={false}
      >
        <MultiSelectEditor
          title="Do you have any pets? (select all that apply)"
          options={QUIZ.sections.pets.questions.pet_ownership.optionsOrder}
          selected={profile.pets.pet_ownership}
          onChange={(selected) =>
            setProfile((p) => ({
              ...p,
              pets: { ...p.pets, pet_ownership: selected },
            }))
          }
          exclusiveWith={[["none", ["dog", "cat", "small_pet", "other"]]]}
          sectionKey="pets"
          questionKey="pet_ownership"
        />

        <MultiSelectEditor
          title="OK living with roommate's pets (select all that apply)"
          options={QUIZ.sections.pets.questions.pet_tolerance.optionsOrder}
          selected={profile.pets.pet_tolerance}
          onChange={(selected) =>
            setProfile((p) => ({
              ...p,
              pets: { ...p.pets, pet_tolerance: selected },
            }))
          }
          exclusiveWith={[
            [
              "no_pets",
              [
                "dog_small",
                "dog_large",
                "cat_ok",
                "small_pets_ok",
                "hypo_only",
                "other_ok",
              ],
            ],
          ]}
          sectionKey="pets"
          questionKey="pet_tolerance"
        />

        <MultiSelectEditor
          title="Allergies (select all that apply)"
          options={QUIZ.sections.pets.questions.pet_allergies.optionsOrder}
          selected={profile.pets.pet_allergies}
          onChange={(selected) =>
            setProfile((p) => ({
              ...p,
              pets: { ...p.pets, pet_allergies: selected },
            }))
          }
          exclusiveWith={[
            [
              "none",
              [
                "allergy_dog",
                "allergy_large_dog",
                "allergy_small_dog",
                "allergy_cat",
                "allergy_small_pets",
                "allergy_other",
              ],
            ],
          ]}
          sectionKey="pets"
          questionKey="pet_allergies"
        />
      </ScrollView>
    ) : (
      <View style={styles.petsContainer}>
        <View style={styles.petSection}>
          <Text style={styles.petSectionTitle}>I have:</Text>
          <View style={styles.petTags}>
            {profile.pets.pet_ownership.includes("none") ? (
              <View style={styles.petTag}>
                <Text style={styles.petTagText}>No pets</Text>
              </View>
            ) : (
              profile.pets.pet_ownership.map((pet) => (
                <View key={pet} style={styles.petTag}>
                  <Text style={styles.petTagText}>
                    {getOptionLabel("pets", "pet_ownership", pet)}
                  </Text>
                </View>
              ))
            )}
          </View>
        </View>

        <View style={styles.petSection}>
          <Text style={styles.petSectionTitle}>OK with roommate's:</Text>
          <View style={styles.petTags}>
            {profile.pets.pet_tolerance.includes("no_pets") ? (
              <View style={styles.petTag}>
                <Text style={styles.petTagText}>Pet-free preferred</Text>
              </View>
            ) : (
              profile.pets.pet_tolerance.map((tolerance) => (
                <View key={tolerance} style={styles.petTag}>
                  <Text style={styles.petTagText}>
                    {getOptionLabel("pets", "pet_tolerance", tolerance)}
                  </Text>
                </View>
              ))
            )}
          </View>
        </View>

        {!profile.pets.pet_allergies.includes("none") && (
          <View style={styles.petSection}>
            <Text style={styles.petSectionTitle}>Allergies:</Text>
            <View style={styles.petTags}>
              {profile.pets.pet_allergies.map((allergy) => (
                <View key={allergy} style={[styles.petTag, styles.allergyTag]}>
                  <Text style={[styles.petTagText, styles.allergyTagText]}>
                    {getOptionLabel("pets", "pet_allergies", allergy)}
                  </Text>
                </View>
              ))}
            </View>
          </View>
        )}
      </View>
    )}
  </View>
);

const SubstanceItem = ({ icon, label, value, status }) => (
  <View style={styles.substanceItem}>
    <View style={styles.substanceHeader}>
      <Text style={styles.substanceIcon}>{icon}</Text>
      <Text style={styles.substanceLabel}>{label}</Text>
    </View>
    <Text
      style={[
        styles.substanceValue,
        status === "good" && styles.substanceValueGood,
        status === "okay" && styles.substanceValueOkay,
      ]}
    >
      {value}
    </Text>
    <View
      style={[
        styles.substanceIndicator,
        status === "good" && styles.substanceIndicatorGood,
        status === "okay" && styles.substanceIndicatorOkay,
      ]}
    />
  </View>
);

/* ===================== STYLES (added header/sticky bits used above) ===================== */
const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: "#F8FAFC" },

  /* Shared section styles */
  modernSection: {
    backgroundColor: "#FFFFFF",
    marginTop: 12,
    borderRadius: 20,
    padding: 24,
    marginHorizontal: 16,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.04,
    shadowRadius: 8,
    elevation: 2,
  },
  modernSectionHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 20,
  },
  sectionTitleContainer: { flexDirection: "row", alignItems: "center" },
  modernSectionTitle: {
    fontSize: 18,
    fontWeight: "700",
    color: "#1F2937",
    marginLeft: 8,
    letterSpacing: -0.2,
  },
  editChip: {
    backgroundColor: "#F3F4F6",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  editChipText: { color: "#6B7280", fontSize: 12, fontWeight: "600" },
  editingContainer: { maxHeight: 420, paddingVertical: 4 },

  /* Basics – Budget & Duration */
  budgetEditorContainer: { marginTop: 8, marginBottom: 12 },
  editorSectionTitle: {
    fontSize: 14,
    fontWeight: "700",
    color: "#1F2937",
    marginBottom: 8,
  },
  budgetInputsContainer: { flexDirection: "row", alignItems: "center", gap: 10 },
  budgetInputWrapper: { flex: 1 },
  budgetInputLabel: { fontSize: 12, color: "#6B7280", marginBottom: 6, fontWeight: "600" },
  budgetInput: {
    backgroundColor: "#FFFFFF",
    borderWidth: 1.5,
    borderColor: "#E5E7EB",
    borderRadius: 12,
    paddingHorizontal: 12,
    paddingVertical: 10,
    fontSize: 14,
    color: "#111827",
  },
  budgetSeparator: { color: "#6B7280", fontSize: 12, fontWeight: "600", paddingHorizontal: 4 },

  durationPickerContainer: { marginTop: 12 },
  durationInputsContainer: { flexDirection: "row", alignItems: "center", gap: 8, marginTop: 6 },
  durationInput: {
    backgroundColor: "#FFFFFF",
    borderWidth: 1.5,
    borderColor: "#E5E7EB",
    borderRadius: 12,
    paddingHorizontal: 12,
    paddingVertical: 10,
    fontSize: 14,
    color: "#111827",
    minWidth: 80,
  },
  durationLabel: { fontSize: 12, color: "#6B7280", fontWeight: "600" },

  detailsContainer: { gap: 16 },
  detailCard: {
    backgroundColor: "#F8FAFC",
    borderRadius: 16,
    padding: 20,
    borderWidth: 1,
    borderColor: "#E2E8F0",
  },
  detailCardTitle: { fontSize: 16, fontWeight: "700", color: "#1F2937", marginBottom: 16 },
  modernDetailRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingVertical: 12,
    borderBottomWidth: 1,
    borderBottomColor: "#E5E7EB",
  },
  modernDetailLeft: { flexDirection: "row", alignItems: "center", gap: 8, flex: 1 },
  modernDetailLabel: { fontSize: 14, fontWeight: "600", color: "#6B7280" },
  modernDetailValue: { fontSize: 14, fontWeight: "600", color: "#111827", textAlign: "right", flex: 1 },

  /* Lifestyle */
  lifestyleGrid: { gap: 16 },
  lifestyleItem: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingVertical: 12,
    borderBottomWidth: 1,
    borderBottomColor: "#F1F5F9",
  },
  lifestyleLabel: { fontSize: 14, fontWeight: "600", color: "#374151", flex: 1 },
  lifestyleValue: { fontSize: 14, fontWeight: "600", color: "#6B7280" },
  scaleContainer: { flexDirection: "row", gap: 4, marginRight: 12 },
  scaleDot: { width: 12, height: 12, borderRadius: 6, backgroundColor: "#E5E7EB" },
  scaleDotActive: { backgroundColor: "#3B82F6" },
  scaleValue: { fontSize: 12, fontWeight: "600", color: "#6B7280", minWidth: 24 },

  /* Substances */
  substancesGrid: { flexDirection: "row", flexWrap: "wrap", gap: 12 },
  substanceItem: {
    flex: 1,
    minWidth: "45%",
    backgroundColor: "#F8FAFC",
    borderWidth: 1,
    borderColor: "#E2E8F0",
    borderRadius: 16,
    padding: 16,
    position: "relative",
  },
  substanceHeader: { flexDirection: "row", alignItems: "center", marginBottom: 8 },
  substanceIcon: { fontSize: 20, marginRight: 8 },
  substanceLabel: { fontSize: 14, fontWeight: "600", color: "#374151" },
  substanceValue: { fontSize: 12, fontWeight: "500", color: "#6B7280" },
  substanceValueGood: { color: "#059669" },
  substanceValueOkay: { color: "#D97706" },
  substanceIndicator: { position: "absolute", top: 12, right: 12, width: 8, height: 8, borderRadius: 4, backgroundColor: "#9CA3AF" },
  substanceIndicatorGood: { backgroundColor: "#10B981" },
  substanceIndicatorOkay: { backgroundColor: "#F59E0B" },
  toggleContainer: { marginTop: 16, flexDirection: "row", alignItems: "center", justifyContent: "space-between" },
  toggleTitle: { fontSize: 14, fontWeight: "600", color: "#1F2937", flex: 1, paddingRight: 12 },
  toggleButton: {
    paddingHorizontal: 14,
    paddingVertical: 8,
    borderRadius: 999,
    backgroundColor: "#F3F4F6",
    borderWidth: 1.5,
    borderColor: "#E5E7EB",
  },
  toggleButtonActive: { backgroundColor: "#10B981", borderColor: "#10B981" },
  toggleButtonText: { fontSize: 12, fontWeight: "700", color: "#374151" },
  toggleButtonTextActive: { color: "#FFFFFF" },

  /* Pets */
  petsContainer: { gap: 20 },
  petSection: { gap: 12 },
  petSectionTitle: { fontSize: 14, fontWeight: "600", color: "#374151" },
  petTags: { flexDirection: "row", flexWrap: "wrap", gap: 8 },
  petTag: {
    backgroundColor: "#FEF3C7",
    borderWidth: 1,
    borderColor: "#FDE68A",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
  },
  petTagText: { fontSize: 12, fontWeight: "500", color: "#92400E" },
  allergyTag: { backgroundColor: "#FEE2E2", borderColor: "#FECACA" },
  allergyTagText: { color: "#991B1B" },
});

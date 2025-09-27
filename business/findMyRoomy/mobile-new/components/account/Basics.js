import { Ionicons } from "@expo/vector-icons";
import * as Haptics from "expo-haptics";
import React, { useEffect, useState } from "react";
import {
  Platform,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View,
  Dimensions,
  Alert,
} from "react-native";

import { QUIZ } from "@/components/Interests_lifestyle";

const SingleSelectEditor = ({
  title,
  options,
  selected,
  onChange,
  sectionKey,
  questionKey,
}) => (
  <View style={styles.singleSelectContainer}>
    <Text style={styles.singleSelectTitle}>{title}</Text>
    <View style={styles.selectOptionsContainer}>
      {options.map((optionKey) => {
        const isSelected = selected === optionKey;
        const label = getOptionLabel(sectionKey, questionKey, optionKey);
        return (
          <TouchableOpacity
  key={optionKey}
  style={[
    styles.selectOption,
    isSelected && styles.selectOptionSelected,
  ]}
  onPress={() => {
    if (Platform.OS === "ios") Haptics.selectionAsync();
    onChange(optionKey);
  }}
  activeOpacity={0.8}
>
  <Text
    style={[
      styles.selectOptionText,
      isSelected && styles.selectOptionTextSelected,
    ]}
  >
    {label}
  </Text>

  {/* Always show the checkmark */}
  <Ionicons
    name="checkmark-circle"
    size={20}
    color={isSelected ? "#3B82F6" : "#CBD5E1"} // blue when selected, gray when not
  />
</TouchableOpacity>

        );
      })}
    </View>
  </View>
);

const formatBudgetRange = (min, max) => {
  if (!min && !max) return "Not set";
  if (min && max) return `$${min.toLocaleString()} - $${max.toLocaleString()}`;
  if (min) return `$${min.toLocaleString()}+`;
  if (max) return `Up to $${max.toLocaleString()}`;
  return "Not set";
};

const formatDuration = (months) => {
  if (!months) return "Open-ended";
  if (months < 12) return `${months} months`;
  const years = Math.floor(months / 12);
  const remainingMonths = months % 12;
  if (remainingMonths === 0) return `${years} year${years > 1 ? "s" : ""}`;
  return `${years}.${
    Math.round(remainingMonths / 6) * 6 === 6 ? "5" : "0"
  } years`;
};

const getOptionLabel = (sectionKey, questionKey, optionKey) => {
  const section = QUIZ.sections[sectionKey];
  if (!section) return optionKey;
  const question = section.questions[questionKey];
  if (!question || !question.options) return optionKey;
  const option = question.options[optionKey];
  return option?.label || optionKey;
};

const { width } = Dimensions.get("window");

/* ===================== BASICS ===================== */
export const Basics = ({
  editing,
  profile,
  setProfile,
  toggleEdit,
  setDirty,
}) => {
  const [initialProfile, setInitialProfile] = useState(null);
  const [localBudgetMin, setLocalBudgetMin] = useState("");
  const [localBudgetMax, setLocalBudgetMax] = useState("");
  const [localLeaseDuration, setLocalLeaseDuration] = useState("");

  // console.log("profile on mount in basics", profile);

  const LEASE_DURATION_MAP = {
    "3_6": 6,
    12: 12,
    "18p": 18,
    mtm: 1, // month-to-month
    flex: 0, // flexible / open-ended
  };


  // Initialize local state when entering edit mode
  useEffect(() => {
    if (editing.basics) {
      setInitialProfile(JSON.parse(JSON.stringify(profile)));
      setLocalBudgetMin(profile.budget_min ? String(profile.budget_min) : "");
      setLocalBudgetMax(profile.budget_max ? String(profile.budget_max) : "");
      setLocalLeaseDuration(
        profile.lease_duration_months !== null && profile.lease_duration_months !== undefined
          ? String(profile.lease_duration_months)
          : ""
      );
      
    }
  }, [
    editing.basics
  ]);

    // Initialize local state when entering edit mode
    useEffect(() => {
      if (editing.basics) {
        setLocalLeaseDuration(
          profile.lease_duration_months
            ? String(profile.lease_duration_months)
            : ""
        );
      }
    }, [
      profile.lease_duration_months,
    ]);

  const validateInputs = () => {
    const min = localBudgetMin ? parseInt(localBudgetMin, 10) : null;
    const max = localBudgetMax ? parseInt(localBudgetMax, 10) : null;

    if (min && max && min >= max) {
      Alert.alert(
        "Invalid Budget Range",
        "Maximum budget must be greater than minimum budget.",
        [{ text: "OK" }]
      );
      return false;
    }

    if (min && min < 0) {
      Alert.alert("Invalid Budget", "Budget cannot be negative.", [
        { text: "OK" },
      ]);
      return false;
    }

    return true;
  };

  const handleToggle = () => {
    if (editing.basics) {
      if (!validateInputs()) return;
  
      const min = localBudgetMin ? parseInt(localBudgetMin, 10) : null;
      const max = localBudgetMax ? parseInt(localBudgetMax, 10) : null;
      
      // Only override lease_duration_months if user entered a specific value in the text input
      const specificMonths = localLeaseDuration ? parseInt(localLeaseDuration, 10) : null;
      
      // Update the profile with budget changes and optional lease duration override
      setProfile((prev) => ({
        ...prev,
        budget_min: min,
        budget_max: max,
        // Only override lease_duration_months if user typed a specific value
        ...(specificMonths ? { lease_duration_months: specificMonths } : {}),
      }));
  
      // Check for changes to mark as dirty
      if (initialProfile) {
        const hasChanges = 
          initialProfile.budget_min !== min ||
          initialProfile.budget_max !== max ||
          (specificMonths && initialProfile.lease_duration_months !== specificMonths) ||
          initialProfile.move_in_selection !== profile.move_in_selection ||
          JSON.stringify(initialProfile.room_preferences) !== JSON.stringify(profile.room_preferences) ||
          initialProfile.lease_duration_months !== profile.lease_duration_months;
        
        if (hasChanges){
          setDirty(true);
          console.log("did change")
        }else{
          console.log("did not change")
        }
        console.log(profile)
      }
    }
    toggleEdit("basics");
  };
  const handleBudgetMinChange = (text) => {
    const cleanText = text.replace(/[^\d]/g, "");
    setLocalBudgetMin(cleanText);
  };

  const handleBudgetMaxChange = (text) => {
    const cleanText = text.replace(/[^\d]/g, "");
    setLocalBudgetMax(cleanText);
  };

  const handleLeaseDurationChange = (text) => {
    const cleanText = text.replace(/[^\d]/g, "");
    if (
      cleanText === "" ||
      (parseInt(cleanText) >= 1 && parseInt(cleanText) <= 60)
    ) {
      setLocalLeaseDuration(cleanText);
    }
  };

  return (
    <View style={styles.modernSection}>
      <View style={styles.modernSectionHeader}>
        <View style={styles.sectionTitleContainer}>
          <View style={styles.iconContainer}>
            <Ionicons name="home-outline" size={22} color="#3B82F6" />
          </View>
          <Text style={styles.modernSectionTitle}>Housing Basics</Text>
        </View>

        <TouchableOpacity
          style={[styles.editChip, editing.basics && styles.editChipActive]}
          onPress={handleToggle}
          activeOpacity={0.8}
        >
          <Ionicons
            name={editing.basics ? "checkmark" : "pencil"}
            size={14}
            color={editing.basics ? "#FFFFFF" : "#6B7280"}
          />
          <Text
            style={[
              styles.editChipText,
              editing.basics && styles.editChipTextActive,
            ]}
          >
            {editing.basics ? "Done" : "Edit"}
          </Text>
        </TouchableOpacity>
      </View>

      {editing.basics ? (
        <ScrollView
          style={styles.editingContainer}
          showsVerticalScrollIndicator={false}
        >
          {/* Budget Range Editor */}
          <View style={styles.budgetEditorContainer}>
            <Text style={styles.editorSectionTitle}>
              Budget Range (Monthly)
            </Text>
            <Text style={styles.editorSectionSubtitle}>
              Set your preferred rent range to find suitable options
            </Text>
            <View style={styles.budgetInputsContainer}>
              <View style={styles.budgetInputWrapper}>
                <Text style={styles.budgetInputLabel}>Minimum</Text>
                <View style={styles.budgetInputContainer}>
                  <Text style={styles.budgetPrefix}>$</Text>
                  <TextInput
                    style={styles.budgetInput}
                    value={localBudgetMin}
                    onChangeText={handleBudgetMinChange}
                    keyboardType="numeric"
                    placeholder="800"
                    placeholderTextColor="#9CA3AF"
                  />
                </View>
              </View>

              <View style={styles.budgetSeparatorContainer}>
                <View style={styles.budgetSeparatorLine} />
              </View>

              <View style={styles.budgetInputWrapper}>
                <Text style={styles.budgetInputLabel}>Maximum</Text>
                <View style={styles.budgetInputContainer}>
                  <Text style={styles.budgetPrefix}>$</Text>
                  <TextInput
                    style={styles.budgetInput}
                    value={localBudgetMax}
                    onChangeText={handleBudgetMaxChange}
                    keyboardType="numeric"
                    placeholder="1500"
                    placeholderTextColor="#9CA3AF"
                  />
                </View>
              </View>
            </View>
          </View>

          {/* Move-in Timeline */}
          <SingleSelectEditor
            title="When are you looking to move?"
            options={
              QUIZ.sections.basics.questions.move_in_timeline.optionsOrder
            }
            selected={profile.move_in_selection}
            onChange={(value) =>
              setProfile((p) => ({ ...p, move_in_selection: value }))
            }
            sectionKey="basics"
            questionKey="move_in_timeline"
          />

          {/* Room Type */}
          <SingleSelectEditor
            title="Room type preference"
            options={QUIZ.sections.basics.questions.room_type.optionsOrder}
            selected={profile.room_preferences?.[0] ?? ""}
            onChange={(value) =>
              setProfile((p) => ({ ...p, room_preferences: [value] }))
            }
            sectionKey="basics"
            questionKey="room_type"
          />

          {/* Lease Duration Choice */}
          <SingleSelectEditor
            title="Preferred lease duration"
            options={QUIZ.sections.basics.questions.lease_duration.optionsOrder}
            selected={
              Object.keys(LEASE_DURATION_MAP).find(
                (key) =>
                  LEASE_DURATION_MAP[key] === profile.lease_duration_months
              ) ?? ""
            }
            onChange={(value) => {
              console.log("\n\nTapped", value, typeof value, "\n\n");
              const choice = LEASE_DURATION_MAP[value];
              console.log("\n\n", choice, "\n\n");
              setProfile((p) => ({
                ...p,
                lease_duration_months: choice ?? null,
              }));
            }}
            sectionKey="basics"
            questionKey="lease_duration"
          />
        </ScrollView>
      ) : (
<View style={styles.detailsContainer}>
  <View style={styles.detailCard}>
    <Text style={styles.detailCardLabel}>Monthly Budget</Text>
    <Text style={styles.detailCardValue}>
      {formatBudgetRange(profile.budget_min, profile.budget_max)}
    </Text>
  </View>

  <View style={styles.detailCard}>
    <Text style={styles.detailCardLabel}>Move-in Timeline</Text>
    <Text style={styles.detailCardValue}>
      {profile.move_in_selection
        ? getOptionLabel("basics", "move_in_timeline", profile.move_in_selection)
        : "Not set"}
    </Text>
  </View>

  <View style={styles.detailCard}>
    <Text style={styles.detailCardLabel}>Room Type</Text>
    <Text style={styles.detailCardValue}>
      {profile.room_preferences?.[0]
        ? getOptionLabel("basics", "room_type", profile.room_preferences[0])
        : "Not set"}
    </Text>
  </View>

  <View style={styles.detailCard}>
    <Text style={styles.detailCardLabel}>Lease Preference</Text>
    <Text style={styles.detailCardValue}>
      {(() => {
        const matchingKey = Object.keys(LEASE_DURATION_MAP).find(
          (key) =>
            LEASE_DURATION_MAP[key] === profile.lease_duration_months
        );
        return matchingKey
          ? getOptionLabel("basics", "lease_duration", matchingKey)
          : "Not set";
      })()}
    </Text>
  </View>
</View>

      )}
    </View>
  );
};

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: "#F8FAFC" },

  /* Shared section styles */
  modernSection: {
    backgroundColor: "#FFFFFF",
    marginTop: 12,
    borderRadius: 24,
    padding: 24,
    marginHorizontal: 16,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.06,
    shadowRadius: 12,
    elevation: 3,
    borderWidth: 1,
    borderColor: "#F1F5F9",
  },

  modernSectionHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 24,
  },

  sectionTitleContainer: {
    flexDirection: "row",
    alignItems: "center",
  },

  iconContainer: {
    backgroundColor: "#EFF6FF",
    padding: 8,
    borderRadius: 12,
    marginRight: 12,
  },

  modernSectionTitle: {
    fontSize: 20,
    fontWeight: "800",
    color: "#1F2937",
    letterSpacing: -0.3,
  },

  editChip: {
    backgroundColor: "#F8FAFC",
    paddingHorizontal: 16,
    paddingVertical: 10,
    borderRadius: 20,
    borderWidth: 2,
    borderColor: "#E2E8F0",
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.05,
    shadowRadius: 4,
    elevation: 2,
  },

  editChipActive: {
    backgroundColor: "#3B82F6",
    borderColor: "#3B82F6",
    shadowColor: "#3B82F6",
    shadowOpacity: 0.25,
  },

  editChipText: {
    color: "#64748B",
    fontSize: 14,
    fontWeight: "700",
  },

  editChipTextActive: {
    color: "#FFFFFF",
  },

  editingContainer: {
    maxHeight: 500,
    paddingVertical: 4,
  },

  /* Budget Editor */
  budgetEditorContainer: {
    marginBottom: 20,
  },

  editorSectionTitle: {
    fontSize: 16,
    fontWeight: "700",
    color: "#1F2937",
    marginBottom: 4,
  },

  editorSectionSubtitle: {
    fontSize: 14,
    color: "#6B7280",
    marginBottom: 16,
    lineHeight: 20,
  },

  budgetInputsContainer: {
    flexDirection: "row",
    alignItems: "flex-end",
    gap: 16,
  },

  budgetInputWrapper: {
    flex: 1,
  },

  budgetInputLabel: {
    fontSize: 13,
    color: "#374151",
    marginBottom: 8,
    fontWeight: "600",
  },

  budgetInputContainer: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#FFFFFF",
    borderWidth: 2,
    borderColor: "#E5E7EB",
    borderRadius: 16,
    paddingHorizontal: 16,
    paddingVertical: 14,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.03,
    shadowRadius: 4,
    elevation: 1,
  },

  budgetPrefix: {
    fontSize: 16,
    fontWeight: "600",
    color: "#6B7280",
    marginRight: 4,
  },

  budgetInput: {
    flex: 1,
    fontSize: 16,
    color: "#111827",
    fontWeight: "600",
    padding: 0,
  },

  budgetSeparatorContainer: {
    alignItems: "center",
    paddingBottom: 14,
  },

  budgetSeparatorLine: {
    width: 20,
    height: 2,
    backgroundColor: "#D1D5DB",
    borderRadius: 1,
  },

  /* Duration Input */
  durationPickerContainer: {
    marginTop: 20,
  },

  durationInputsContainer: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
  },

  durationInput: {
    backgroundColor: "#FFFFFF",
    borderWidth: 2,
    borderColor: "#E5E7EB",
    borderRadius: 16,
    paddingHorizontal: 16,
    paddingVertical: 14,
    fontSize: 16,
    color: "#111827",
    fontWeight: "600",
    width: 100,
    textAlign: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.03,
    shadowRadius: 4,
    elevation: 1,
  },

  durationLabel: {
    fontSize: 14,
    color: "#6B7280",
    fontWeight: "600",
  },

  /* Read-Only Card */
  detailsContainer: {
    flexDirection: "row",
    flexWrap: "wrap",
    justifyContent: "space-between", // pushes items into 2-column grid
    marginTop: 12,
  },


detailCard: {
  width: "48%",              // each card takes half row
  marginBottom: 16,          // spacing between rows
  backgroundColor: "#F8FAFC",
  borderRadius: 16,
  padding: 16,
  borderWidth: 1,
  borderColor: "#E2E8F0",
  shadowColor: "#000",
  shadowOffset: { width: 0, height: 2 },
  shadowOpacity: 0.05,
  shadowRadius: 4,
  elevation: 2,
},
  
  detailCardLabel: {
    fontSize: 13,
    fontWeight: "600",
    color: "#6B7280",
    marginBottom: 6,
  },

  detailCardTitle: {
    fontSize: 18,
    fontWeight: "700",
    color: "#1F2937",
    marginBottom: 20,
  },

  modernDetailRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingVertical: 16,
    borderBottomWidth: 1,
    borderBottomColor: "#E5E7EB",
  },

  modernDetailLeft: {
    flexDirection: "row",
    alignItems: "center",
    gap: 12,
    flex: 1,
  },
  detailCardValue: {
    fontSize: 15,
    fontWeight: "700",
    color: "#111827",
  },
  modernDetailLabel: {
    fontSize: 15,
    fontWeight: "600",
    color: "#374151",
  },

  modernDetailValue: {
    fontSize: 15,
    fontWeight: "700",
    color: "#111827",
    textAlign: "right",
    flex: 1,
  },

  /* Single Select Editor */
  singleSelectContainer: {
    marginTop: 20,
  },

  singleSelectTitle: {
    fontSize: 16,
    fontWeight: "700",
    color: "#1F2937",
    marginBottom: 4,
  },

  selectOptionsContainer: {
    gap: 12,
    marginTop: 12,
  },

  selectOption: {
    flexDirection: "row",        // ✅ put label + checkmark in a row
    alignItems: "center",        // ✅ vertically center them
    justifyContent: "space-between", // ✅ push label left, checkmark right
    paddingVertical: 12,
    paddingHorizontal: 18,
    borderRadius: 999,
    borderWidth: 1.5,
    borderColor: "#CBD5E1",
    backgroundColor: "#F9FAFB",
  },
  

  selectOptionSelected: {
    borderColor: "#3B82F6",
    backgroundColor: "#EFF6FF",
    shadowColor: "#3B82F6",
    shadowOpacity: 0.1,
  },

  selectOptionText: {
    fontSize: 15,
    fontWeight: "500",
    color: "#374151",
    flexShrink: 1,   // ✅ prevent text from pushing the checkmark
    flexWrap: "wrap" // ✅ allow long labels to wrap
  },
  

  selectOptionTextSelected: {
    color: "#1D4ED8",
    fontWeight: "700",
  },
});

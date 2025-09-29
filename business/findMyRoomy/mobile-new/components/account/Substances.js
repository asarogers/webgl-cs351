import { Ionicons } from "@expo/vector-icons";
import * as Haptics from "expo-haptics";
import React, { useEffect } from "react";
import {
  Platform,
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
  Animated,
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
            activeOpacity={0.7}
          >
            <View style={styles.selectOptionContent}>
              <Text
                style={[
                  styles.selectOptionText,
                  isSelected && styles.selectOptionTextSelected,
                ]}
              >
                {label}
              </Text>
              <View
                style={[
                  styles.checkmarkContainer,
                  isSelected && styles.checkmarkContainerSelected,
                ]}
              >
                <Ionicons
                  name="checkmark"
                  size={16}
                  color={isSelected ? "#FFFFFF" : "transparent"}
                />
              </View>
            </View>
          </TouchableOpacity>
        );
      })}
    </View>
  </View>
);

const getOptionLabel = (sectionKey, questionKey, optionKey) => {
  const section = QUIZ.sections[sectionKey];
  if (!section) return optionKey;
  const question = section.questions[questionKey];
  if (!question || !question.options) return optionKey;
  const option = question.options[optionKey];
  return option?.label || optionKey;
};

const SubstanceItem = ({ icon, label, value, status }) => (
  <View
    style={[
      styles.substanceItem,
      status === "good" && styles.substanceItemGood,
      status === "okay" && styles.substanceItemOkay,
    ]}
  >
    <View style={styles.substanceHeader}>
      <View
        style={[
          styles.substanceIconContainer,
          status === "good" && styles.substanceIconContainerGood,
          status === "okay" && styles.substanceIconContainerOkay,
        ]}
      >
        <Text style={styles.substanceIcon}>{icon}</Text>
      </View>
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
        styles.substanceStatusDot,
        status === "good" && styles.substanceStatusDotGood,
        status === "okay" && styles.substanceStatusDotOkay,
      ]}
    />
  </View>
);

export const Substances = ({
  editing,
  profile,
  setProfile,
  toggleEdit,
  setDirty,
}) => {
  useEffect(() => {
    // console.log("profile", profile);
  }, [profile]); // runs every time profile changes

  return (
    <View style={styles.modernSection}>
      <View style={styles.modernSectionHeader}>
        <View style={styles.sectionTitleContainer}>
          <View style={styles.sectionIconContainer}>
            <Ionicons name="wine-outline" size={22} color="#F59E0B" />
          </View>
          <Text style={styles.modernSectionTitle}>Substances</Text>
        </View>
        <TouchableOpacity
          style={[styles.editChip, editing.substances && styles.editChipActive]}
          onPress={() => toggleEdit("substances")}
          activeOpacity={0.8}
        >
          <Ionicons
            name={editing.substances ? "checkmark" : "pencil"}
            size={14}
            color={editing.substances ? "#FFFFFF" : "#6B7280"}
            style={styles.editChipIcon}
          />
          <Text
            style={[
              styles.editChipText,
              editing.substances && styles.editChipTextActive,
            ]}
          >
            {editing.substances ? "Done" : "Edit"}
          </Text>
        </TouchableOpacity>
      </View>

      {editing.substances ? (
        <ScrollView
          style={styles.editingContainer}
          showsVerticalScrollIndicator={false}
        >
          <SingleSelectEditor
            title="Smoking / vaping at home"
            options={
              QUIZ.sections.substances?.questions.smoking_policy.optionsOrder
            }
            selected={profile?.substances?.smoking_policy}
            onChange={(value) => {
              setProfile((p) => ({
                ...p,
                substances: { ...p.substances, smoking_policy: value },
              }));
              setDirty(true);
            }}
            sectionKey="substances"
            questionKey="smoking_policy"
          />

          <SingleSelectEditor
            title="Alcohol at home"
            options={
              QUIZ.sections.substances?.questions.alcohol_use.optionsOrder
            }
            selected={profile.substances?.alcohol_use}
            onChange={(value) => {
              setProfile((p) => ({
                ...p,
                substances: { ...p.substances, alcohol_use: value },
              }));
              setDirty(true);
            }}
            sectionKey="substances"
            questionKey="alcohol_use"
          />

          <SingleSelectEditor
            title="Cannabis (where legal)"
            options={
              QUIZ.sections.substances?.questions.cannabis_policy.optionsOrder
            }
            selected={profile.substances?.cannabis_policy}
            onChange={(value) => {
              setProfile((p) => ({
                ...p,
                substances: { ...p.substances, cannabis_policy: value },
              }));
              setDirty(true);
            }}
            sectionKey="substances"
            questionKey="cannabis_policy"
          />

          <View style={styles.toggleContainer}>
            <View style={styles.toggleHeader}>
              <Ionicons name="home-outline" size={18} color="#6B7280" />
              <Text style={styles.toggleTitle}>
                I want a substance-free household
              </Text>
            </View>
            <TouchableOpacity
              style={[
                styles.toggleButton,
                profile.substances?.substance_free_preference &&
                  styles.toggleButtonActive,
              ]}
              onPress={async () => {
                if (Platform.OS === "ios") await Haptics.selectionAsync();
                setProfile((p) => ({
                  ...p,
                  substances: {
                    ...p.substances,
                    substance_free_preference:
                      !p.substances?.substance_free_preference,
                  },
                }));
              }}
              activeOpacity={0.8}
            >
              <View
                style={[
                  styles.toggleSlider,
                  profile.substances?.substance_free_preference &&
                    styles.toggleSliderActive,
                ]}
              />
            </TouchableOpacity>
          </View>
        </ScrollView>
      ) : (
        <View style={styles.substancesGrid}>
          <SubstanceItem
            icon="🚭"
            label="Smoking"
            value={getOptionLabel(
              "substances",
              "smoking_policy",
              profile?.substances?.smoking_policy
            )}
            status={
              profile?.substances?.smoking_policy === "non_smoker"
                ? "good"
                : profile?.substances?.smoking_policy === "smoker_outside"
                ? "okay"
                : "neutral"
            }
          />
          <SubstanceItem
            icon="🍷"
            label="Alcohol"
            value={getOptionLabel(
              "substances",
              "alcohol_use",
              profile.substances?.alcohol_use
            )}
            status={
              profile.substances?.alcohol_use === "social_drinker"
                ? "good"
                : "neutral"
            }
          />
          <SubstanceItem
            icon="🌿"
            label="Cannabis"
            value={getOptionLabel(
              "substances",
              "cannabis_policy",
              profile.substances?.cannabis_policy
            )}
            status={
              profile.substances?.cannabis_policy === "no_cannabis"
                ? "good"
                : profile.substances?.cannabis_policy === "ok_outside"
                ? "okay"
                : "neutral"
            }
          />

          {profile.substances?.substance_free_preference && (
            <View style={styles.substanceFreeCard}>
              <View style={styles.substanceFreeIcon}>
                <Ionicons name="shield-checkmark" size={20} color="#10B981" />
              </View>
              <Text style={styles.substanceFreeText}>
                Substance-free household preferred
              </Text>
            </View>
          )}
        </View>
      )}
    </View>
  );
};

const styles = StyleSheet.create({
  /* Shared section styles */
  modernSection: {
    backgroundColor: "#FFFFFF",
    marginTop: 12,
    borderRadius: 24,
    padding: 24,
    marginHorizontal: 16,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08,
    shadowRadius: 12,
    elevation: 3,
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
  sectionIconContainer: {
    width: 40,
    height: 40,
    borderRadius: 20,
    backgroundColor: "#FEF3C7",
    alignItems: "center",
    justifyContent: "center",
    marginRight: 12,
  },
  modernSectionTitle: {
    fontSize: 20,
    fontWeight: "700",
    color: "#1F2937",
    letterSpacing: -0.3,
  },
  editChip: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#F8FAFC",
    paddingHorizontal: 16,
    paddingVertical: 10,
    borderRadius: 20,
    borderWidth: 1,
    borderColor: "#E2E8F0",
  },
  editChipActive: {
    backgroundColor: "#10B981",
    borderColor: "#10B981",
  },
  editChipIcon: {
    marginRight: 6,
  },
  editChipText: {
    color: "#6B7280",
    fontSize: 13,
    fontWeight: "600",
  },
  editChipTextActive: {
    color: "#FFFFFF",
  },
  editingContainer: {
    maxHeight: 420,
    paddingVertical: 4,
  },

  /* Single Select Editor */
  singleSelectContainer: {
    marginBottom: 24,
  },
  singleSelectTitle: {
    fontSize: 16,
    fontWeight: "600",
    color: "#1F2937",
    marginBottom: 12,
  },
  selectOptionsContainer: {
    gap: 8,
  },
  selectOption: {
    backgroundColor: "#F8FAFC",
    borderWidth: 1.5,
    borderColor: "#E2E8F0",
    borderRadius: 16,
    padding: 16,
    transition: "all 0.2s ease",
  },
  selectOptionSelected: {
    backgroundColor: "#EFF6FF",
    borderColor: "#3B82F6",
  },
  selectOptionContent: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
  },
  selectOptionText: {
    fontSize: 14,
    fontWeight: "500",
    color: "#6B7280",
    flex: 1,
  },
  selectOptionTextSelected: {
    color: "#1F2937",
    fontWeight: "600",
  },
  checkmarkContainer: {
    width: 24,
    height: 24,
    borderRadius: 12,
    backgroundColor: "#E2E8F0",
    alignItems: "center",
    justifyContent: "center",
  },
  checkmarkContainerSelected: {
    backgroundColor: "#3B82F6",
  },

  /* Substances Grid */
  substancesGrid: {
    flexDirection: "row",
    flexWrap: "wrap",
    gap: 12,
  },
  substanceItem: {
    flex: 1,
    minWidth: "45%",
    backgroundColor: "#F8FAFC",
    borderWidth: 1.5,
    borderColor: "#E2E8F0",
    borderRadius: 20,
    padding: 18,
    position: "relative",
  },
  substanceItemGood: {
    backgroundColor: "#F0FDF4",
    borderColor: "#BBF7D0",
  },
  substanceItemOkay: {
    backgroundColor: "#FFFBEB",
    borderColor: "#FED7AA",
  },
  substanceHeader: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 12,
  },
  substanceIconContainer: {
    width: 32,
    height: 32,
    borderRadius: 16,
    backgroundColor: "#F1F5F9",
    alignItems: "center",
    justifyContent: "center",
    marginRight: 10,
  },
  substanceIconContainerGood: {
    backgroundColor: "#DCFCE7",
  },
  substanceIconContainerOkay: {
    backgroundColor: "#FEF3C7",
  },
  substanceIcon: {
    fontSize: 16,
  },
  substanceLabel: {
    fontSize: 14,
    fontWeight: "600",
    color: "#374151",
    flex: 1,
  },
  substanceValue: {
    fontSize: 13,
    fontWeight: "500",
    color: "#6B7280",
    lineHeight: 18,
  },
  substanceValueGood: {
    color: "#059669",
  },
  substanceValueOkay: {
    color: "#D97706",
  },
  substanceStatusDot: {
    position: "absolute",
    top: 16,
    right: 16,
    width: 8,
    height: 8,
    borderRadius: 4,
    backgroundColor: "#CBD5E1",
  },
  substanceStatusDotGood: {
    backgroundColor: "#10B981",
  },
  substanceStatusDotOkay: {
    backgroundColor: "#F59E0B",
  },

  /* Substance-free card */
  substanceFreeCard: {
    width: "100%",
    backgroundColor: "#F0FDF4",
    borderWidth: 1.5,
    borderColor: "#BBF7D0",
    borderRadius: 20,
    padding: 18,
    flexDirection: "row",
    alignItems: "center",
    marginTop: 4,
  },
  substanceFreeIcon: {
    width: 32,
    height: 32,
    borderRadius: 16,
    backgroundColor: "#DCFCE7",
    alignItems: "center",
    justifyContent: "center",
    marginRight: 12,
  },
  substanceFreeText: {
    fontSize: 14,
    fontWeight: "600",
    color: "#059669",
    flex: 1,
  },

  /* Toggle */
  toggleContainer: {
    marginTop: 20,
    backgroundColor: "#F8FAFC",
    borderRadius: 16,
    padding: 16,
    borderWidth: 1,
    borderColor: "#E2E8F0",
  },
  toggleHeader: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 12,
  },
  toggleTitle: {
    fontSize: 15,
    fontWeight: "600",
    color: "#1F2937",
    marginLeft: 8,
    flex: 1,
  },
  toggleButton: {
    width: 52,
    height: 28,
    borderRadius: 14,
    backgroundColor: "#E5E7EB",
    padding: 2,
    justifyContent: "center",
  },
  toggleButtonActive: {
    backgroundColor: "#10B981",
  },
  toggleSlider: {
    width: 24,
    height: 24,
    borderRadius: 12,
    backgroundColor: "#FFFFFF",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 3,
    elevation: 2,
  },
  toggleSliderActive: {
    transform: [{ translateX: 24 }],
  },
});

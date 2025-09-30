import { Ionicons } from "@expo/vector-icons";
import * as Haptics from "expo-haptics";
import React, { useEffect, useState } from "react";
import {
  Alert,
  Animated,
  Dimensions,
  Image,
  Platform,
  ScrollView,
  StyleSheet,
  Switch,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";

import authService from "@/database/authService";

const { width } = Dimensions.get("window");

const MultiSelectEditor = ({
  title,
  options,
  selected = [],
  onChange,
  exclusiveWith = [],
  sectionKey,
  questionKey,
  otherText = {},
  onOtherTextChange,
}) => {
  const hasOther = options.some((opt) => opt.includes("other"));
  const isOtherSelected = selected.some((s) => s.includes("other"));

  return (
    <View style={styles.multiSelectContainer}>
      <Text style={styles.multiSelectTitle}>{title}</Text>
      <View style={styles.selectableTagsContainer}>
        {options.map((optionKey) => {
          const isSelected = selected.includes(optionKey);
          const label = getOptionLabel(sectionKey, questionKey, optionKey);
          return (
            <TouchableOpacity
              key={optionKey}
              style={[styles.choiceTag, isSelected && styles.choiceTagSelected]}
              onPress={() => {
                if (Platform.OS === "ios") Haptics.selectionAsync();
                let nextSelected = isSelected
                  ? selected.filter((i) => i !== optionKey)
                  : [...selected, optionKey];

                exclusiveWith.forEach(([exclusive, conflicting]) => {
                  if (optionKey === exclusive) {
                    nextSelected = nextSelected.filter(
                      (i) => !conflicting.includes(i)
                    );
                  } else if (conflicting.includes(optionKey)) {
                    nextSelected = nextSelected.filter((i) => i !== exclusive);
                  }
                });

                onChange(nextSelected);
              }}
              activeOpacity={0.7}
            >
              <Text
                style={[
                  styles.choiceTagText,
                  isSelected && styles.choiceTagTextSelected,
                ]}
              >
                {label}
              </Text>
              {isSelected && (
                <View style={styles.checkmarkContainer}>
                  <Ionicons name="checkmark-circle" size={16} color="#10B981" />
                </View>
              )}
            </TouchableOpacity>
          );
        })}
      </View>
      {hasOther && isOtherSelected && (
        <View style={styles.otherInputContainer}>
          <Ionicons
            name="create-outline"
            size={18}
            color="#6B7280"
            style={styles.otherInputIcon}
          />
          <TextInput
            style={styles.otherInput}
            placeholder="Please specify..."
            placeholderTextColor="#9CA3AF"
            value={otherText[questionKey] || ""}
            onChangeText={(text) => onOtherTextChange?.(questionKey, text)}
            autoCapitalize="sentences"
            maxLength={100}
          />
        </View>
      )}
    </View>
  );
};

const getOptionLabel = (sectionKey, questionKey, optionKey) => {
  const section = QUIZ.sections[sectionKey];
  if (!section) return optionKey;
  const question = section.questions[questionKey];
  if (!question || !question.options) return optionKey;
  const option = question.options[optionKey];
  return option?.label || optionKey;
};

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

export const Amenities = ({
  editing,
  profile,
  setProfile,
  toggleEdit,
  setDirty,
}) => (
  <View style={styles.modernSection}>
    <View style={styles.modernSectionHeader}>
      <View style={styles.sectionTitleContainer}>
        <View style={[styles.iconContainer, { backgroundColor: "#8B5CF6" }]}>
          <Ionicons name="checkmark-circle-outline" size={18} color="#FFFFFF" />
        </View>
        <View>
          <Text style={styles.modernSectionTitle}>Amenities</Text>
          <Text style={styles.sectionSubtitle}>Your preferences</Text>
        </View>
      </View>
      <TouchableOpacity
        style={[styles.editChip, editing.amenities && styles.editChipActive]}
        onPress={() => toggleEdit("amenities")}
        activeOpacity={0.7}
      >
        <Ionicons
          name={editing.amenities ? "checkmark" : "pencil"}
          size={12}
          color={editing.amenities ? "#FFFFFF" : "#6B7280"}
        />
        <Text
          style={[
            styles.editChipText,
            editing.amenities && styles.editChipTextActive,
          ]}
        >
          {editing.amenities ? "Done" : "Edit"}
        </Text>
      </TouchableOpacity>
    </View>

    {editing.amenities ? (
      <ScrollView
        style={styles.editingContainer}
        showsVerticalScrollIndicator={false}
      >
        <SingleSelectEditor
          title="Bathroom"
          options={
            QUIZ.sections.amenities?.questions.bathroom_pref.optionsOrder
          }
          selected={profile.amenities?.bathroom_pref}
          onChange={(value) => {
            setProfile((p) => ({
              ...p,
              amenities: { ...p.amenities, bathroom_pref: value },
            }));
            setDirty(true);
          }}
          sectionKey="amenities"
          questionKey="bathroom_pref"
        />

        <SingleSelectEditor
          title="Laundry"
          options={QUIZ.sections.amenities?.questions.laundry.optionsOrder}
          selected={profile.amenities?.laundry}
          onChange={(value) => {
            setProfile((p) => ({
              ...p,
              amenities: { ...p.amenities, laundry: value },
            }));
            setDirty(true);
          }}
          sectionKey="amenities"
          questionKey="laundry"
        />

        <SingleSelectEditor
          title="Parking"
          options={QUIZ.sections.amenities?.questions.parking.optionsOrder}
          selected={profile.amenities?.parking}
          onChange={(value) => {
            setProfile((p) => ({
              ...p,
              amenities: { ...p.amenities, parking: value },
            }));
            setDirty(true);
          }}
          sectionKey="amenities"
          questionKey="parking"
        />

        <SingleSelectEditor
          title="Furnishing"
          options={QUIZ.sections.amenities?.questions.furnishing.optionsOrder}
          selected={profile.amenities?.furnishing}
          onChange={(value) => {
            setProfile((p) => ({
              ...p,
              amenities: { ...p.amenities, furnishing: value },
            }));
            setDirty(true);
          }}
          sectionKey="amenities"
          questionKey="furnishing"
        />

        <MultiSelectEditor
          title="Nice-to-haves"
          options={QUIZ.sections.amenities?.questions.extras.optionsOrder}
          selected={profile.amenities?.extras || []}
          onChange={(selected) => {
            setProfile((p) => ({
              ...p,
              amenities: { ...p.amenities, extras: selected },
            }));
            setDirty(true);
          }}
          sectionKey="amenities"
          questionKey="extras"
        />
      </ScrollView>
    ) : (
      <>
        <View style={styles.amenitiesGrid}>
          <AmenityItem
            icon="🚿"
            label="Bathroom"
            value={getOptionLabel(
              "amenities",
              "bathroom_pref",
              profile?.amenities?.bathroom_pref
            )}
          />
          <AmenityItem
            icon="🧺"
            label="Laundry"
            value={getOptionLabel(
              "amenities",
              "laundry",
              profile?.amenities?.laundry
            )}
          />
          <AmenityItem
            icon="🚗"
            label="Parking"
            value={getOptionLabel(
              "amenities",
              "parking",
              profile?.amenities?.parking
            )}
          />
          <AmenityItem
            icon="🛋️"
            label="Furnishing"
            value={getOptionLabel(
              "amenities",
              "furnishing",
              profile?.amenities?.furnishing
            )}
          />
        </View>

        {profile.amenities?.extras && profile?.amenities?.extras.length > 0 && (
          <View style={styles.extrasSection}>
            <Text style={styles.extrasSectionTitle}>Nice-to-haves</Text>
            <View style={styles.extrasTags}>
              {profile.amenities?.extras.map((extra) => (
                <View key={extra} style={styles.extraTag}>
                  <Text style={styles.extraTagText}>
                    {getOptionLabel("amenities", "extras", extra)}
                  </Text>
                </View>
              ))}
            </View>
          </View>
        )}
      </>
    )}
  </View>
);

/* ===================== PRESENTATIONAL HELPERS ===================== */

const AmenityItem = ({ icon, label, value }) => (
  <View style={styles.amenityItem}>
    <View style={styles.amenityIconContainer}>
      <Text style={styles.amenityIcon}>{icon}</Text>
    </View>
    <View style={styles.amenityContent}>
      <Text style={styles.amenityLabel}>{label}</Text>
      <Text style={styles.amenityValue}>{value}</Text>
    </View>
  </View>
);

/* ===================== ENHANCED STYLES ===================== */
const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: "#F8FAFC" },

  /* Section wrapper */
  modernSection: {
    backgroundColor: "#FFFFFF",
    marginTop: 16,
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
  iconContainer: {
    width: 40,
    height: 40,
    borderRadius: 12,
    justifyContent: "center",
    alignItems: "center",
    marginRight: 12,
  },
  modernSectionTitle: {
    fontSize: 20,
    fontWeight: "800",
    color: "#1E293B",
    letterSpacing: -0.3,
  },
  sectionSubtitle: {
    fontSize: 13,
    color: "#64748B",
    fontWeight: "500",
  },

  /* Edit toggle chip */
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
  },
  editChipActive: {
    backgroundColor: "#3B82F6",
    borderColor: "#3B82F6",
  },
  editChipText: { color: "#64748B", fontSize: 13, fontWeight: "700" },
  editChipTextActive: { color: "#FFFFFF" },

  /* Editing scroll container */
  editingContainer: { maxHeight: 420, paddingVertical: 8 },

  /* ================= MULTI-SELECT ================= */
  multiSelectContainer: { marginBottom: 24 },
  multiSelectTitle: {
    fontSize: 15,
    fontWeight: "600",
    color: "#111827",
    marginBottom: 12,
  },
  selectableTagsContainer: {
    flexDirection: "row",
    flexWrap: "wrap",
    gap: 10,
  },
  choiceTag: {
    flexDirection: "row",
    alignItems: "center",
    borderWidth: 2,
    borderColor: "#E5E7EB",
    borderRadius: 12,
    paddingHorizontal: 14,
    paddingVertical: 10,
    backgroundColor: "#FFFFFF",
    gap: 6,
  },
  choiceTagSelected: {
    backgroundColor: "#F0FDF4",
    borderColor: "#10B981",
  },
  choiceTagText: {
    fontSize: 14,
    fontWeight: "500",
    color: "#374151",
  },
  choiceTagTextSelected: {
    color: "#047857",
    fontWeight: "600",
  },
  checkmarkContainer: { marginLeft: 2 },

  otherInputContainer: {
    flexDirection: "row",
    alignItems: "center",
    marginTop: 12,
    backgroundColor: "#F9FAFB",
    borderRadius: 12,
    borderWidth: 1.5,
    borderColor: "#E5E7EB",
    paddingHorizontal: 12,
    paddingVertical: 4,
  },
  otherInputIcon: { marginRight: 8 },
  otherInput: {
    flex: 1,
    fontSize: 14,
    color: "#111827",
    paddingVertical: 10,
    fontWeight: "500",
  },

  /* ================= SINGLE-SELECT ================= */
  singleSelectContainer: { marginBottom: 24 },
  singleSelectTitle: {
    fontSize: 16,
    fontWeight: "600",
    color: "#1F2937",
    marginBottom: 12,
  },
  selectOptionsContainer: { gap: 8 },
  selectOption: {
    backgroundColor: "#F8FAFC",
    borderWidth: 1.5,
    borderColor: "#E2E8F0",
    borderRadius: 16,
    padding: 16,
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
  checkmarkContainerSingle: {
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

  /* ================= AMENITIES VIEW ================= */
  amenitiesGrid: { gap: 4 },
  amenityItem: {
    flexDirection: "row",
    alignItems: "center",
    paddingVertical: 16,
    borderBottomWidth: 1,
    borderBottomColor: "#F1F5F9",
  },
  amenityIconContainer: {
    width: 40,
    height: 40,
    borderRadius: 12,
    backgroundColor: "#F8FAFC",
    justifyContent: "center",
    alignItems: "center",
    marginRight: 16,
  },
  amenityIcon: { fontSize: 20 },
  amenityContent: { flex: 1 },
  amenityLabel: {
    fontSize: 15,
    fontWeight: "700",
    color: "#374151",
    marginBottom: 2,
  },
  amenityValue: { fontSize: 13, fontWeight: "600", color: "#64748B" },

  extrasSection: {
    marginTop: 24,
    paddingTop: 24,
    borderTopWidth: 2,
    borderTopColor: "#F1F5F9",
  },
  extrasSectionTitle: {
    fontSize: 15,
    fontWeight: "700",
    color: "#374151",
    marginBottom: 16,
  },
  extrasTags: { flexDirection: "row", flexWrap: "wrap", gap: 8 },
  extraTag: {
    backgroundColor: "#F0F9FF",
    borderWidth: 1,
    borderColor: "#BAE6FD",
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 20,
  },
  extraTagText: { fontSize: 12, fontWeight: "600", color: "#0C4A6E" },
});

import React, { useEffect, useState } from "react";
import {
  View,
  Text,
  StyleSheet,
  ScrollView,
  TouchableOpacity,
  TextInput,
  Platform,
} from "react-native";
import { Ionicons } from "@expo/vector-icons";
import * as Haptics from "expo-haptics";
import { QUIZ } from "@/components/Interests_lifestyle";

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
  const hasOther = options.some(opt => opt.includes('other'));
  const isOtherSelected = selected.some(s => s.includes('other'));

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
          <Ionicons name="create-outline" size={18} color="#6B7280" style={styles.otherInputIcon} />
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

export const Pets = ({
  editing,
  profile,
  setProfile,
  toggleEdit,
  setDirty,
}) => {
  const [otherText, setOtherText] = useState({
    pet_ownership: "",
    pet_tolerance: "",
    pet_allergies: "",
  });

  // Load existing "other" text from profile when component mounts or profile changes
  useEffect(() => {
    if (profile?.pets) {
      const ownership = profile.pets.pet_ownership || [];
      const tolerance = profile.pets.pet_tolerance || [];
      const allergies = profile.pets.pet_allergies || [];

      setOtherText({
        pet_ownership: ownership.find(item => typeof item === 'object' && item.other)?.other || "",
        pet_tolerance: tolerance.find(item => typeof item === 'object' && item.other)?.other || "",
        pet_allergies: allergies.find(item => typeof item === 'object' && item.other)?.other || "",
      });
    }
  }, [profile?.pets]);

  const handleOtherTextChange = (questionKey, text) => {
    setOtherText((prev) => ({ ...prev, [questionKey]: text }));
    
    // Update the profile with the new structure
    setProfile((p) => {
      const currentPets = p.pets || {};
      const currentArray = currentPets[questionKey] || [];
      
      // Remove any existing "other" objects and filter out plain "other" strings
      const filteredArray = currentArray.filter(item => {
        if (typeof item === 'string') {
          return !item.includes('other');
        }
        return !item.other;
      });

      // Add back the "other" string and the object with custom text if there's text
      const hasOtherSelected = currentArray.some(item => 
        typeof item === 'string' && item.includes('other')
      );

      let updatedArray = [...filteredArray];
      if (hasOtherSelected) {
        // Find the original "other" key
        const otherKey = currentArray.find(item => 
          typeof item === 'string' && item.includes('other')
        );
        updatedArray.push(otherKey);
        
        if (text.trim()) {
          updatedArray.push({ other: text });
        }
      }

      return {
        ...p,
        pets: {
          ...currentPets,
          [questionKey]: updatedArray,
        },
      };
    });
    setDirty(true);
  };

  // Helper function to get display text for an item
  const getDisplayText = (item, sectionKey, questionKey) => {
    if (typeof item === 'object' && item.other) {
      return item.other;
    }
    if (typeof item === 'string') {
      return getOptionLabel(sectionKey, questionKey, item);
    }
    return item;
  };

  // Helper function to get selection array for MultiSelectEditor
  const getSelectionArray = (array) => {
    if (!array) return [];
    return array.filter(item => typeof item === 'string');
  };

  return (
    <View style={styles.modernSection}>
      <View style={styles.modernSectionHeader}>
        <View style={styles.sectionTitleContainer}>
          <View style={styles.sectionIconContainer}>
            <Ionicons name="paw" size={22} color="#F97316" />
          </View>
          <View>
            <Text style={styles.modernSectionTitle}>Pets</Text>
            <Text style={styles.sectionSubtitle}>Your pet preferences</Text>
          </View>
        </View>

        <TouchableOpacity
          style={[styles.editButton, editing.pets && styles.editButtonActive]}
          onPress={() => toggleEdit("pets")}
          activeOpacity={0.7}
        >
          <Ionicons
            name={editing.pets ? "checkmark-circle" : "create-outline"}
            size={20}
            color={editing.pets ? "#FFFFFF" : "#6B7280"}
          />
          <Text
            style={[
              styles.editButtonText,
              editing.pets && styles.editButtonTextActive,
            ]}
          >
            {editing.pets ? "Save" : "Edit"}
          </Text>
        </TouchableOpacity>
      </View>

      {editing.pets ? (
        <ScrollView
          style={styles.editingContainer}
          showsVerticalScrollIndicator={false}
        >
          <MultiSelectEditor
            title="Do you have any pets?"
            options={QUIZ.sections.pets.questions.pet_ownership.optionsOrder}
            selected={getSelectionArray(profile?.pets?.pet_ownership)}
            onChange={(selected) => {
              const existingOther = (profile?.pets?.pet_ownership || []).find(
                item => typeof item === 'object' && item.other
              );
              const newArray = [...selected];
              if (existingOther) {
                newArray.push(existingOther);
              }
              setProfile((p) => ({
                ...p,
                pets: { ...p.pets, pet_ownership: newArray },
              }));
              setDirty(true);
            }}
            exclusiveWith={[["none", ["dog", "cat", "small_pet", "other"]]]}
            sectionKey="pets"
            questionKey="pet_ownership"
            otherText={otherText}
            onOtherTextChange={handleOtherTextChange}
          />

          <MultiSelectEditor
            title="OK with roommate's pets?"
            options={QUIZ.sections.pets.questions.pet_tolerance.optionsOrder}
            selected={getSelectionArray(profile?.pets?.pet_tolerance)}
            onChange={(selected) => {
              const existingOther = (profile?.pets?.pet_tolerance || []).find(
                item => typeof item === 'object' && item.other
              );
              const newArray = [...selected];
              if (existingOther) {
                newArray.push(existingOther);
              }
              setProfile((p) => ({
                ...p,
                pets: { ...p.pets, pet_tolerance: newArray },
              }));
              setDirty(true);
            }}
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
            otherText={otherText}
            onOtherTextChange={handleOtherTextChange}
          />

          <MultiSelectEditor
            title="Any pet allergies?"
            options={QUIZ.sections.pets.questions.pet_allergies.optionsOrder}
            selected={getSelectionArray(profile?.pets?.pet_allergies)}
            onChange={(selected) => {
              const existingOther = (profile?.pets?.pet_allergies || []).find(
                item => typeof item === 'object' && item.other
              );
              const newArray = [...selected];
              if (existingOther) {
                newArray.push(existingOther);
              }
              setProfile((p) => ({
                ...p,
                pets: { ...p.pets, pet_allergies: newArray },
              }));
              setDirty(true);
            }}
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
            otherText={otherText}
            onOtherTextChange={handleOtherTextChange}
          />
        </ScrollView>
      ) : (
        <View style={styles.petsContainer}>
          <View style={styles.petSection}>
            <View style={styles.petSectionHeader}>
              <View style={styles.iconBadge}>
                <Ionicons name="home" size={14} color="#F97316" />
              </View>
              <Text style={styles.petSectionTitle}>My Pets</Text>
            </View>
            <View style={styles.petTags}>
              {(profile?.pets?.pet_ownership ?? []).some(item => 
                (typeof item === 'string' && item === 'none')
              ) ? (
                <View style={styles.petTag}>
                  <Ionicons name="close-circle" size={14} color="#92400E" style={styles.tagIcon} />
                  <Text style={styles.petTagText}>No pets</Text>
                </View>
              ) : (profile?.pets?.pet_ownership ?? []).length > 0 ? (
                <>
                  {(profile?.pets?.pet_ownership ?? [])
                    .filter(item => typeof item === 'string' || (typeof item === 'object' && item.other))
                    .map((pet, index) => {
                      const displayText = getDisplayText(pet, "pets", "pet_ownership");
                      return (
                        <View key={`${typeof pet === 'object' ? 'other' : pet}-${index}`} style={styles.petTag}>
                          <Ionicons name="paw" size={14} color="#F97316" style={styles.tagIcon} />
                          <Text style={styles.petTagText}>{displayText}</Text>
                        </View>
                      );
                    })}
                </>
              ) : (
                <Text style={styles.emptyText}>Not specified</Text>
              )}
            </View>
          </View>

          <View style={styles.divider} />

          <View style={styles.petSection}>
            <View style={styles.petSectionHeader}>
              <View style={styles.iconBadge}>
                <Ionicons name="checkmark-circle" size={14} color="#10B981" />
              </View>
              <Text style={styles.petSectionTitle}>Comfortable With</Text>
            </View>
            <View style={styles.petTags}>
              {(profile?.pets?.pet_tolerance ?? []).some(item => 
                (typeof item === 'string' && item === 'no_pets')
              ) ? (
                <View style={[styles.petTag, styles.neutralTag]}>
                  <Ionicons name="ban" size={14} color="#6B7280" style={styles.tagIcon} />
                  <Text style={[styles.petTagText, styles.neutralTagText]}>
                    Pet-free preferred
                  </Text>
                </View>
              ) : (profile?.pets?.pet_tolerance ?? []).length > 0 ? (
                (profile?.pets?.pet_tolerance ?? [])
                  .filter(item => typeof item === 'string' || (typeof item === 'object' && item.other))
                  .map((tolerance, index) => {
                    const displayText = getDisplayText(tolerance, "pets", "pet_tolerance");
                    return (
                      <View key={`${typeof tolerance === 'object' ? 'other' : tolerance}-${index}`} style={[styles.petTag, styles.toleranceTag]}>
                        <Ionicons name="checkmark" size={14} color="#10B981" style={styles.tagIcon} />
                        <Text style={[styles.petTagText, styles.toleranceTagText]}>
                          {displayText}
                        </Text>
                      </View>
                    );
                  })
              ) : (
                <Text style={styles.emptyText}>Not specified</Text>
              )}
            </View>
          </View>

          {!(profile?.pets?.pet_allergies ?? []).some(item => 
            (typeof item === 'string' && item === 'none')
          ) && (profile?.pets?.pet_allergies ?? []).length > 0 && (
              <>
                <View style={styles.divider} />
                <View style={styles.petSection}>
                  <View style={styles.petSectionHeader}>
                    <View style={[styles.iconBadge, styles.allergyBadge]}>
                      <Ionicons name="alert-circle" size={14} color="#DC2626" />
                    </View>
                    <Text style={styles.petSectionTitle}>Allergies</Text>
                  </View>
                  <View style={styles.petTags}>
                    {(profile?.pets?.pet_allergies ?? [])
                      .filter(item => typeof item === 'string' || (typeof item === 'object' && item.other))
                      .map((allergy, index) => {
                        const displayText = getDisplayText(allergy, "pets", "pet_allergies");
                        return (
                          <View key={`${typeof allergy === 'object' ? 'other' : allergy}-${index}`} style={[styles.petTag, styles.allergyTag]}>
                            <Ionicons name="warning" size={14} color="#DC2626" style={styles.tagIcon} />
                            <Text style={[styles.petTagText, styles.allergyTagText]}>
                              {displayText}
                            </Text>
                          </View>
                        );
                      })}
                  </View>
                </View>
              </>
            )}
        </View>
      )}
    </View>
  );
};

const styles = StyleSheet.create({
  modernSection: {
    backgroundColor: "#FFFFFF",
    marginTop: 16,
    borderRadius: 24,
    padding: 20,
    marginHorizontal: 16,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08,
    shadowRadius: 12,
    elevation: 4,
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
    flex: 1,
  },
  sectionIconContainer: {
    width: 48,
    height: 48,
    borderRadius: 16,
    backgroundColor: "#FFF7ED",
    alignItems: "center",
    justifyContent: "center",
    marginRight: 12,
  },
  modernSectionTitle: {
    fontSize: 20,
    fontWeight: "700",
    color: "#111827",
    letterSpacing: -0.5,
  },
  sectionSubtitle: {
    fontSize: 13,
    color: "#9CA3AF",
    marginTop: 2,
    fontWeight: "500",
  },
  editButton: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#F9FAFB",
    paddingHorizontal: 16,
    paddingVertical: 10,
    borderRadius: 14,
    gap: 6,
  },
  editButtonActive: {
    backgroundColor: "#10B981",
  },
  editButtonText: {
    color: "#6B7280",
    fontSize: 14,
    fontWeight: "600",
  },
  editButtonTextActive: {
    color: "#FFFFFF",
  },
  editingContainer: {
    maxHeight: 460,
  },
  multiSelectContainer: {
    marginBottom: 24,
  },
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
  checkmarkContainer: {
    marginLeft: 2,
  },
  petsContainer: {
    gap: 0,
  },
  petSection: {
    paddingVertical: 4,
  },
  petSectionHeader: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 12,
    gap: 8,
  },
  iconBadge: {
    width: 28,
    height: 28,
    borderRadius: 8,
    backgroundColor: "#FFF7ED",
    alignItems: "center",
    justifyContent: "center",
  },
  allergyBadge: {
    backgroundColor: "#FEF2F2",
  },
  petSectionTitle: {
    fontSize: 15,
    fontWeight: "600",
    color: "#111827",
  },
  petTags: {
    flexDirection: "row",
    flexWrap: "wrap",
    gap: 8,
  },
  petTag: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#FFF7ED",
    borderWidth: 1.5,
    borderColor: "#FDBA74",
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 10,
    gap: 6,
  },
  toleranceTag: {
    backgroundColor: "#F0FDF4",
    borderColor: "#86EFAC",
  },
  toleranceTagText: {
    color: "#047857",
  },
  neutralTag: {
    backgroundColor: "#F9FAFB",
    borderColor: "#D1D5DB",
  },
  neutralTagText: {
    color: "#4B5563",
  },
  tagIcon: {
    marginRight: -2,
  },
  petTagText: {
    fontSize: 13,
    fontWeight: "600",
    color: "#C2410C",
  },
  allergyTag: {
    backgroundColor: "#FEF2F2",
    borderColor: "#FCA5A5",
  },
  allergyTagText: {
    color: "#DC2626",
  },
  emptyText: {
    fontSize: 13,
    color: "#9CA3AF",
    fontStyle: "italic",
  },
  divider: {
    height: 1,
    backgroundColor: "#F3F4F6",
    marginVertical: 20,
  },
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
  otherInputIcon: {
    marginRight: 8,
  },
  otherInput: {
    flex: 1,
    fontSize: 14,
    color: "#111827",
    paddingVertical: 10,
    fontWeight: "500",
    outlineStyle: "none",
  },
});
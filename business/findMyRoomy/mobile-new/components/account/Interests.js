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
// Use:
import {
  getZipCode,
  setUseAuto,
} from "@/components/account/location/requestLocation";

import { INTERESTS } from "@/components/Interests_lifestyle";

const { width } = Dimensions.get("window");

const getAllInterests = () => {
    const allInterests = [];
    INTERESTS.groupsOrder.forEach((groupKey) => {
      const group = INTERESTS.groups[groupKey];
      group.optionsOrder.forEach((optionKey) => {
        allInterests.push(group.options[optionKey]);
      });
    });
    return allInterests;
  };


export const Interests = ({
    editing,
    profile,
    updateProfile,
    toggleEdit,
  }) => (
    <View style={styles.modernSection}>
      <View style={styles.modernSectionHeader}>
        <View style={styles.sectionTitleContainer}>
          <View style={[styles.iconContainer, { backgroundColor: "#F59E0B" }]}>
            <Ionicons name="heart-outline" size={18} color="#FFFFFF" />
          </View>
          <View>
            <Text style={styles.modernSectionTitle}>Interests</Text>
            <Text style={styles.sectionSubtitle}>What you love</Text>
          </View>
        </View>
        <TouchableOpacity
          style={[styles.editChip, editing.interests && styles.editChipActive]}
          onPress={() => toggleEdit("interests")}
          activeOpacity={0.7}
        >
          <Ionicons
            name={editing.interests ? "checkmark" : "pencil"}
            size={12}
            color={editing.interests ? "#FFFFFF" : "#6B7280"}
          />
          <Text
            style={[
              styles.editChipText,
              editing.interests && styles.editChipTextActive,
            ]}
          >
            {editing.interests ? "Done" : "Edit"}
          </Text>
        </TouchableOpacity>
      </View>
  
      {!editing.interests ? (
        <View style={styles.modernTagsContainer}>
          {profile.interests.length ? (
            profile.interests.map((interest, index) => (
              <Animated.View
                key={interest}
                style={[
                  styles.modernTag,
                  {
                    transform: [{ scale: 1 }],
                    opacity: 1,
                  },
                ]}
              >
                <Text style={styles.modernTagText}>{interest}</Text>
              </Animated.View>
            ))
          ) : (
            <View style={styles.emptyState}>
              <Ionicons name="heart-outline" size={32} color="#D1D5DB" />
              <Text style={styles.emptyStateText}>No interests selected yet</Text>
              <Text style={styles.emptyStateSubtext}>Tap edit to add some</Text>
            </View>
          )}
        </View>
      ) : (
        <>
          <Text style={styles.subtleLabel}>
            Tap to select or deselect interests
          </Text>
          <View style={styles.selectableTagsContainer}>
            {getAllInterests().map((interest) => {
              const selected = profile.interests.includes(interest);
              return (
                <TouchableOpacity
                  key={interest}
                  style={[styles.choiceTag, selected && styles.choiceTagSelected]}
                  onPress={async () => {
                    if (Platform.OS === "ios") await Haptics.selectionAsync();
                    updateProfile((p) => ({
                      ...p,
                      interests: selected
                        ? p.interests.filter((i) => i !== interest)
                        : [...p.interests, interest],
                    }));
                  }}
                  activeOpacity={0.7}
                >
                  <Text
                    style={[
                      styles.choiceTagText,
                      selected && styles.choiceTagTextSelected,
                    ]}
                  >
                    {interest}
                  </Text>
                  {selected && (
                    <Ionicons name="checkmark" size={14} color="#FFFFFF" />
                  )}
                </TouchableOpacity>
              );
            })}
          </View>
        </>
      )}
    </View>
  );

  const styles = StyleSheet.create({
    container: { flex: 1, backgroundColor: "#F8FAFC" },
  
    /* Sticky header - Enhanced */
    stickyHeader: {
      backgroundColor: "#FFFFFF",
      paddingHorizontal: 20,
      paddingTop: 60,
      paddingBottom: 20,
      flexDirection: "row",
      justifyContent: "space-between",
      alignItems: "center",
      borderBottomWidth: 1,
      borderBottomColor: "#F1F5F9",
      shadowColor: "#000",
      shadowOffset: { width: 0, height: 2 },
      shadowOpacity: 0.05,
      shadowRadius: 8,
      elevation: 3,
    },
    headerLeft: { flexDirection: "row", alignItems: "center", gap: 16 },
    saveStatusPill: {
      flexDirection: "row",
      alignItems: "center",
      gap: 8,
      backgroundColor: "#F8FAFC",
      paddingHorizontal: 12,
      paddingVertical: 8,
      borderRadius: 20,
      borderWidth: 1,
      borderColor: "#E2E8F0",
    },
    saveStatusPillSaving: {
      backgroundColor: "#EFF6FF",
      borderColor: "#DBEAFE",
    },
    saveStatusPillDirty: {
      backgroundColor: "#FFFBEB",
      borderColor: "#FDE68A",
    },
    saveStatusText: { fontSize: 13, color: "#475569", fontWeight: "600" },
    dot: { width: 8, height: 8, borderRadius: 4, backgroundColor: "#10B981" },
    dotPulse: {
      backgroundColor: "#3B82F6",
      transform: [{ scale: 1 }],
    },
    saveButton: {
      flexDirection: "row",
      gap: 8,
      backgroundColor: "#3B82F6",
      paddingHorizontal: 16,
      paddingVertical: 12,
      borderRadius: 16,
      shadowColor: "#3B82F6",
      shadowOffset: { width: 0, height: 2 },
      shadowOpacity: 0.2,
      shadowRadius: 4,
      elevation: 3,
    },
    saveButtonDisabled: { opacity: 0.6, shadowOpacity: 0.1 },
    saveButtonText: { color: "#FFFFFF", fontSize: 14, fontWeight: "700" },
    headerTitle: {
      fontSize: 28,
      fontWeight: "800",
      color: "#1E293B",
      letterSpacing: -1,
    },
  
  
  
    /* Sections - Enhanced */
    modernSection: {
      backgroundColor: "#FFFFFF",
      marginTop: 16,
      borderRadius: 24,
      padding: 28,
      marginHorizontal: 20,
      shadowColor: "#000",
      shadowOffset: { width: 0, height: 4 },
      shadowOpacity: 0.08,
      shadowRadius: 12,
      elevation: 4,
    },
    modernSectionHeader: {
      flexDirection: "row",
      justifyContent: "space-between",
      alignItems: "flex-start",
      marginBottom: 24,
    },
    sectionTitleContainer: {
      flexDirection: "row",
      alignItems: "center",
      flex: 1,
    },
    iconContainer: {
      width: 40,
      height: 40,
      borderRadius: 12,
      backgroundColor: "#3B82F6",
      justifyContent: "center",
      alignItems: "center",
      marginRight: 16,
      shadowColor: "#3B82F6",
      shadowOffset: { width: 0, height: 2 },
      shadowOpacity: 0.2,
      shadowRadius: 4,
    },
    modernSectionTitle: {
      fontSize: 20,
      fontWeight: "800",
      color: "#1E293B",
      letterSpacing: -0.3,
      marginBottom: 2,
    },
    sectionSubtitle: {
      fontSize: 13,
      color: "#64748B",
      fontWeight: "500",
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
      shadowOffset: { width: 0, height: 1 },
      shadowOpacity: 0.05,
      shadowRadius: 2,
      elevation: 1,
    },
    editChipActive: {
      backgroundColor: "#3B82F6",
      borderColor: "#3B82F6",
      shadowColor: "#3B82F6",
      shadowOpacity: 0.2,
    },
    editChipText: { color: "#64748B", fontSize: 13, fontWeight: "700" },
    editChipTextActive: { color: "#FFFFFF" },
  
    /* Photos - Enhanced */
    modernPhotoGrid: { flexDirection: "row", flexWrap: "wrap", gap: 16 },
    modernPhotoContainer: {
      width: (width - 120) / 3,
      aspectRatio: 1,
      borderRadius: 20,
      overflow: "hidden",
      position: "relative",
      shadowColor: "#000",
      shadowOffset: { width: 0, height: 4 },
      shadowOpacity: 0.12,
      shadowRadius: 8,
      elevation: 4,
    },
    photoTouchable: { width: "100%", height: "100%", position: "relative" },
    modernPhotoImg: { width: "100%", height: "100%" },
    photoRemoveButton: {
      position: "absolute",
      top: 8,
      right: 8,
      backgroundColor: "rgba(239, 68, 68, 0.95)",
      width: 26,
      height: 26,
      borderRadius: 13,
      justifyContent: "center",
      alignItems: "center",
      shadowColor: "#EF4444",
      shadowOffset: { width: 0, height: 2 },
      shadowOpacity: 0.3,
      shadowRadius: 4,
    },
    primaryBadge: {
      position: "absolute",
      bottom: 8,
      left: 8,
      backgroundColor: "rgba(16, 185, 129, 0.95)",
      paddingHorizontal: 8,
      paddingVertical: 4,
      borderRadius: 8,
    },
    primaryBadgeText: { color: "#FFFFFF", fontSize: 10, fontWeight: "700" },
    addPhotoContainer: {
      width: (width - 120) / 3,
      aspectRatio: 1,
      borderRadius: 20,
      justifyContent: "center",
      alignItems: "center",
      borderWidth: 2,
      borderStyle: "dashed",
      borderColor: "#CBD5E1",
      backgroundColor: "#F8FAFC",
    },
    addPhotoIcon: {
      width: 48,
      height: 48,
      borderRadius: 24,
      backgroundColor: "#EBF4FF",
      justifyContent: "center",
      alignItems: "center",
      marginBottom: 8,
    },
    addPhotoIconActive: {
      backgroundColor: "#DBEAFE",
      shadowColor: "#3B82F6",
      shadowOffset: { width: 0, height: 2 },
      shadowOpacity: 0.1,
      shadowRadius: 4,
    },
    addPhotoLabel: {
      fontSize: 12,
      fontWeight: "700",
      color: "#64748B",
      marginBottom: 4,
    },
    photoCounter: {
      backgroundColor: "#E2E8F0",
      paddingHorizontal: 8,
      paddingVertical: 2,
      borderRadius: 12,
    },
    photoCounterText: { fontSize: 10, fontWeight: "600", color: "#475569" },
  
    /* About - Enhanced */
    textareaContainer: { position: "relative" },
    modernTextarea: {
      backgroundColor: "#F8FAFC",
      borderWidth: 2,
      borderColor: "#E2E8F0",
      borderRadius: 20,
      padding: 20,
      fontSize: 16,
      color: "#1E293B",
      minHeight: 120,
      lineHeight: 24,
      fontWeight: "500",
    },
    textareaFooter: {
      flexDirection: "row",
      justifyContent: "flex-end",
      alignItems: "center",
      marginTop: 12,
    },
    characterCount: {
      fontSize: 12,
      color: "#94A3B8",
      fontWeight: "600",
      backgroundColor: "#F1F5F9",
      paddingHorizontal: 8,
      paddingVertical: 4,
      borderRadius: 8,
    },
    aboutText: {
      fontSize: 16,
      lineHeight: 28,
      color: "#475569",
      fontWeight: "500",
      fontStyle: "italic",
    },
  
    /* Interests - Enhanced */
    modernTagsContainer: { flexDirection: "row", flexWrap: "wrap", gap: 12 },
    modernTag: {
      backgroundColor: "#EBF4FF",
      borderWidth: 2,
      borderColor: "#BFDBFE",
      paddingHorizontal: 16,
      paddingVertical: 10,
      borderRadius: 24,
      flexDirection: "row",
      alignItems: "center",
      gap: 6,
      shadowColor: "#3B82F6",
      shadowOffset: { width: 0, height: 2 },
      shadowOpacity: 0.08,
      shadowRadius: 4,
      elevation: 2,
    },
    modernTagText: { color: "#1E40AF", fontSize: 14, fontWeight: "700" },
    emptyState: {
      alignItems: "center",
      paddingVertical: 32,
      backgroundColor: "#F8FAFC",
      borderRadius: 16,
      marginTop: 8,
    },
    emptyStateText: {
      fontSize: 16,
      fontWeight: "600",
      color: "#64748B",
      marginTop: 12,
    },
    emptyStateSubtext: {
      fontSize: 13,
      color: "#94A3B8",
      marginTop: 4,
      fontWeight: "500",
    },
    subtleLabel: {
      color: "#64748B",
      fontSize: 13,
      marginBottom: 16,
      fontWeight: "600",
    },
    selectableTagsContainer: { flexDirection: "row", flexWrap: "wrap", gap: 12 },
    choiceTag: {
      flexDirection: "row",
      alignItems: "center",
      gap: 8,
      paddingHorizontal: 16,
      paddingVertical: 10,
      borderRadius: 24,
      borderWidth: 2,
      borderColor: "#E2E8F0",
      backgroundColor: "#FFFFFF",
      shadowColor: "#000",
      shadowOffset: { width: 0, height: 1 },
      shadowOpacity: 0.05,
      shadowRadius: 2,
      elevation: 1,
    },
    choiceTagSelected: {
      backgroundColor: "#3B82F6",
      borderColor: "#3B82F6",
      shadowColor: "#3B82F6",
      shadowOffset: { width: 0, height: 4 },
      shadowOpacity: 0.2,
      shadowRadius: 6,
      elevation: 4,
      transform: [{ scale: 1.02 }],
    },
    choiceTagText: { color: "#475569", fontSize: 14, fontWeight: "700" },
    choiceTagTextSelected: { color: "#FFFFFF" },
  
    /* Amenities - Enhanced */
    editingContainer: { maxHeight: 420, paddingVertical: 8 },
    amenitiesGrid: { gap: 4 },
    amenityItem: {
      flexDirection: "row",
      alignItems: "center",
      paddingVertical: 16,
      paddingHorizontal: 4,
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
      shadowColor: "#0EA5E9",
      shadowOffset: { width: 0, height: 1 },
      shadowOpacity: 0.05,
      shadowRadius: 2,
    },
    // Add these to your existing styles object:
  
    locationToggleContainer: {
      flexDirection: "row",
      justifyContent: "space-between",
      alignItems: "center",
      marginTop: 16,
      paddingHorizontal: 16,
      paddingVertical: 12,
      backgroundColor: "#F9FAFB",
      borderRadius: 8,
      borderWidth: 1,
      borderColor: "#E5E7EB",
    },
  
    toggleLabel: {
      fontSize: 14,
      color: "#374151",
      fontWeight: "500",
    },
  
    locationDisplay: {
      flexDirection: "row",
      alignItems: "center",
      gap: 8,
      backgroundColor: "#F1F5F9",
      borderColor: "#CBD5E1",
    },
  
    locationDisplayText: {
      fontSize: 16,
      color: "#64748B",
      fontWeight: "600",
      flex: 1,
    },
    extraTagText: { fontSize: 12, fontWeight: "600", color: "#0C4A6E" },
  });
// AccountScreen.tsx — Simplified, no caching, single fetch

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

import authService from "@/database/authService";
const { width } = Dimensions.get("window");


export const AboutSection = ({
    editing,
    profile,
    updateProfile,
    toggleEdit,
  }) => (
    <View style={styles.modernSection}>
      <View style={styles.modernSectionHeader}>
        <View style={styles.sectionTitleContainer}>
          <View style={[styles.iconContainer, { backgroundColor: "#10B981" }]}>
            <Ionicons name="person-outline" size={18} color="#FFFFFF" />
          </View>
          <View>
            <Text style={styles.modernSectionTitle}>About Me</Text>
            <Text style={styles.sectionSubtitle}>Tell your story</Text>
          </View>
        </View>
        <TouchableOpacity
          style={[styles.editChip, editing.about && styles.editChipActive]}
          onPress={() => toggleEdit("about")}
          activeOpacity={0.7}
        >
          <Ionicons
            name={editing.about ? "checkmark" : "pencil"}
            size={12}
            color={editing.about ? "#FFFFFF" : "#6B7280"}
          />
          <Text
            style={[
              styles.editChipText,
              editing.about && styles.editChipTextActive,
            ]}
          >
            {editing.about ? "Done" : "Edit"}
          </Text>
        </TouchableOpacity>
      </View>
  
      {editing.about ? (
        <View style={styles.textareaContainer}>
          <TextInput
            style={styles.modernTextarea}
            multiline
            value={profile.about}
            onChangeText={(t) => updateProfile((p) => ({ ...p, about: t }))}
            placeholder="Tell people about yourself, your interests, what makes you unique..."
            placeholderTextColor="#9CA3AF"
            textAlignVertical="top"
            maxLength={500}
          />
          <View style={styles.textareaFooter}>
            <Text style={styles.characterCount}>
              {(profile.about || "").length}/500
            </Text>
          </View>
        </View>
      ) : (
        <Text style={styles.aboutText}>
          {profile.about ||
            "Add something about yourself to help others get to know you better."}
        </Text>
      )}
    </View>
  );

  const styles = StyleSheet.create({
    /* About Section */
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
  });
  
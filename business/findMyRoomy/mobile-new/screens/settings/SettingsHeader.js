import React from "react";
import { View, Text, StyleSheet, TouchableOpacity, Platform } from "react-native";
import { Ionicons } from "@expo/vector-icons";

const SettingsHeader = ({
  title,
  subtitle,
  onBack,
  showBack = true,
  children, // optional: for custom right-side actions
  style,
}) => (
  <View style={[styles.header, style]}>
    <View style={styles.headerContent}>
      {showBack && (
        <TouchableOpacity
          style={styles.backButton}
          onPress={onBack}
          activeOpacity={0.7}
        >
          <Ionicons name="chevron-back" size={24} color="#fff" />
        </TouchableOpacity>
      )}
      <View style={styles.headerTextContainer}>
        <Text style={styles.headerTitle}>{title}</Text>
        {!!subtitle && <Text style={styles.headerSubtitle}>{subtitle}</Text>}
      </View>
      {children}
    </View>
  </View>
);

const styles = StyleSheet.create({
  header: {
    backgroundColor: "#1F2937",
    paddingBottom: Platform.OS === "ios" ? 24 : 20,
    shadowColor: "#1F2937",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.2,
    shadowRadius: 12,
    elevation: 8,
  },
  headerContent: {
    flexDirection: "row",
    alignItems: "center",
    paddingHorizontal: 20,
    paddingTop: Platform.OS === "ios" ? 16 : 10,
  },
  backButton: {
    width: 40,
    height: 40,
    borderRadius: 20,
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    justifyContent: "center",
    alignItems: "center",
    marginRight: 16,
  },
  headerTextContainer: {
    flex: 1,
  },
  headerTitle: {
    color: "#fff",
    fontSize: 24,
    fontWeight: "700",
    marginBottom: 2,
  },
  headerSubtitle: {
    color: "rgba(255, 255, 255, 0.8)",
    fontSize: 14,
    fontWeight: "400",
  },
});

export default SettingsHeader;

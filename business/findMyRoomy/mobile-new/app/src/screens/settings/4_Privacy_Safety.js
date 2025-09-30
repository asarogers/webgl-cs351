import {
  Ionicons,
  MaterialCommunityIcons,
  MaterialIcons,
} from "@expo/vector-icons";
import React, { useEffect, useState } from "react";
import {
  Alert,
  Modal,
  ScrollView,
  StyleSheet,
  Switch,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import authService from "../../../../database/authService";
import BlockedReportedUsersScreen from "./Reported_Blocked_Users";
import SettingsHeader from "./SettingsHeader";
import TermsAndConditions from "./TermsAndConditions";
import PrivacyPolicy from "./Privacy";

const colors = {
  primary: "#3B82F6",
  primaryHover: "#2563EB",
  mapMarkerBlue: "#60A5FA",
  navy: "#1F2937",
  slate: "#374151",
  grayMedium: "#6B7280",
  grayLight: "#E5E7EB",
  background: "#F8FAFC",
  card: "#FFFFFF",
  cardDark: "#111827",
  tagGreen: "#10B981",
  tagSlate: "#64748B",
  tagBlue: "#3B82F6",
  statusOnline: "#22C55E",
  statusAvailable: "#3B82F6",
  priceGreen: "#22C55E",
  error: "#EF4444",
  warning: "#F59E0B",
};

export const PrivacyAndSafety = ({ setPage }) => {
  const [profileHidden, setProfileHidden] = useState(false);
  const [isLoadingProfileHidden, setIsLoadingProfileHidden] = useState(true);
  const [showFlaggedUsers, setShowFlaggedUsers] = useState(false);
  const [showTerms, setShowTerms] = useState(false);
  const [showPrivacy, setShowPrivacy] = useState(false);

  useEffect(() => {
    let mounted = true;
    async function loadProfileVisibility() {
      setIsLoadingProfileHidden(true);
      const isVisible = await authService.getProfileVisibility();
      if (mounted && typeof isVisible === "boolean") {
        setProfileHidden(!isVisible);
      }
      setIsLoadingProfileHidden(false);
    }
    loadProfileVisibility();
    return () => {
      mounted = false;
    };
  }, []);

  const handleHideProfile = async (value) => {
    setProfileHidden(value);
    try {
      await authService.setProfileVisibility(!value);
      Alert.alert(
        value ? "Profile Hidden" : "Profile Visible",
        value
          ? "Your profile is now hidden from search and discovery."
          : "Your profile is now visible and can be discovered by other users."
      );
    } catch (error) {
      Alert.alert("Error", "Failed to update profile visibility. Try again.");
      setProfileHidden(!value);
    }
  };

  return (
    <SafeAreaView style={styles.container}>
      <SettingsHeader
        title="Privacy & Safety"
        subtitle="Control your privacy and account safety"
        onBack={() => setPage("home")}
      />

      <ScrollView
        style={styles.scrollView}
        contentContainerStyle={{ paddingBottom: 40 }}
        showsVerticalScrollIndicator={false}
      >
        {/* Account Safety Section */}
        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Account Safety</Text>
          <TouchableOpacity
            style={styles.actionCard}
            onPress={() => setShowFlaggedUsers(true)}
            activeOpacity={0.7}
          >
            <View
              style={[styles.iconContainer, { backgroundColor: "#FEF2F2" }]}
            >
              <MaterialIcons name="block" size={22} color={colors.error} />
            </View>
            <View style={styles.contentContainer}>
              <Text style={styles.actionLabel}>
                Manage Blocked/Reported Users
              </Text>
              <Text style={styles.actionSubtitle}>
                Review and manage users you've blocked or reported
              </Text>
            </View>
            <Ionicons
              name="chevron-forward"
              size={20}
              color={colors.grayMedium}
            />
          </TouchableOpacity>
        </View>

        {/* Profile Visibility Section */}
        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Profile Visibility</Text>
          <View style={styles.toggleCard}>
            <View
              style={[
                styles.iconContainer,
                { backgroundColor: profileHidden ? "#FEF3F2" : "#EFF6FF" },
              ]}
            >
              <MaterialCommunityIcons
                name={profileHidden ? "eye-off-outline" : "eye-outline"}
                size={22}
                color={profileHidden ? colors.error : colors.primary}
              />
            </View>
            <View style={styles.contentContainer}>
              <Text style={styles.toggleTitle}>
                {profileHidden ? "Profile Hidden" : "Profile Visible"}
              </Text>
              <Text style={styles.toggleSubtitle}>
                {profileHidden
                  ? "Your profile is hidden from search and cannot be discovered by other users."
                  : "Your profile is visible in search and discoverable by others."}
              </Text>
            </View>
            <Switch
              value={profileHidden}
              onValueChange={handleHideProfile}
              trackColor={{ false: colors.grayLight, true: colors.primary }}
              thumbColor={profileHidden ? "#FFFFFF" : "#FFFFFF"}
              ios_backgroundColor={colors.grayLight}
              disabled={isLoadingProfileHidden}
            />
          </View>
        </View>

        {/* Legal Documents Section */}
        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Legal Documents</Text>

          <TouchableOpacity
            style={styles.linkCard}
            onPress={() => setShowTerms(true)}
            activeOpacity={0.7}
          >
            <View
              style={[styles.iconContainer, { backgroundColor: "#F0F9FF" }]}
            >
              <Ionicons
                name="document-text-outline"
                size={20}
                color={colors.primary}
              />
            </View>
            <View style={styles.contentContainer}>
              <Text style={styles.linkLabel}>Terms and Conditions</Text>
              <Text style={styles.linkSubtitle}>
                View our complete terms of service
              </Text>
            </View>
            <View style={styles.viewIndicator}>
              <Ionicons
                name="chevron-forward"
                size={18}
                color={colors.grayMedium}
              />
            </View>
          </TouchableOpacity>

          <TouchableOpacity
            style={styles.linkCard}
            onPress={() => setShowPrivacy(true)}
            activeOpacity={0.7}
          >
            <View
              style={[styles.iconContainer, { backgroundColor: "#ECFDF5" }]}
            >
              <MaterialCommunityIcons
                name="shield-outline"
                size={20}
                color={colors.tagGreen}
              />
            </View>
            <View style={styles.contentContainer}>
              <Text style={styles.linkLabel}>Privacy Policy</Text>
              <Text style={styles.linkSubtitle}>
                View our privacy and data protection policy
              </Text>
            </View>
            <View style={styles.viewIndicator}>
              <Ionicons
                name="chevron-forward"
                size={18}
                color={colors.grayMedium}
              />
            </View>
          </TouchableOpacity>
        </View>

        {/* Additional Safety Tips */}
        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Safety Tips</Text>
          <View style={styles.tipCard}>
            <View
              style={[styles.iconContainer, { backgroundColor: "#FEF3F2" }]}
            >
              <MaterialCommunityIcons
                name="lightbulb-outline"
                size={20}
                color={colors.warning}
              />
            </View>
            <View style={styles.contentContainer}>
              <Text style={styles.tipTitle}>Stay Safe Online</Text>
              <Text style={styles.tipText}>
                Never share personal information like passwords, financial
                details, or home address with other users.
              </Text>
            </View>
          </View>
        </View>
      </ScrollView>

      {/* Modals - Added presentationStyle="fullScreen" */}
      <Modal
        visible={showFlaggedUsers}
        animationType="slide"
        presentationStyle="fullScreen"
        onRequestClose={() => setShowFlaggedUsers(false)}
      >
        <BlockedReportedUsersScreen onBack={() => setShowFlaggedUsers(false)} />
      </Modal>

      <Modal
        visible={showTerms}
        animationType="slide"
        presentationStyle="fullScreen"
        onRequestClose={() => setShowTerms(false)}
      >
        <TermsAndConditions onBack={() => setShowTerms(false)} />
      </Modal>

      <Modal
        visible={showPrivacy}
        animationType="slide"
        presentationStyle="fullScreen"
        onRequestClose={() => setShowPrivacy(false)}
      >
        <PrivacyPolicy onBack={() => setShowPrivacy(false)} />
      </Modal>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: colors.background },
  scrollView: { flex: 1, paddingHorizontal: 20, marginTop: 20 },
  section: { marginBottom: 32 },
  sectionTitle: {
    fontSize: 13,
    fontWeight: "700",
    color: colors.slate,
    textTransform: "uppercase",
    marginBottom: 16,
    letterSpacing: 0.8,
    marginLeft: 4,
  },
  actionCard: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: colors.card,
    borderRadius: 16,
    padding: 20,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  toggleCard: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: colors.card,
    borderRadius: 16,
    padding: 20,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  linkCard: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: colors.card,
    borderRadius: 16,
    padding: 20,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  tipCard: {
    flexDirection: "row",
    alignItems: "flex-start",
    backgroundColor: colors.card,
    borderRadius: 16,
    padding: 20,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  iconContainer: {
    width: 44,
    height: 44,
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
    marginRight: 16,
  },
  contentContainer: { flex: 1 },
  actionLabel: { fontSize: 16, fontWeight: "600", color: colors.navy },
  actionSubtitle: { fontSize: 13, color: colors.grayMedium },
  toggleTitle: { fontSize: 16, fontWeight: "700", color: colors.navy },
  toggleSubtitle: { fontSize: 13, color: colors.grayMedium },
  linkLabel: { fontSize: 16, fontWeight: "600", color: colors.navy },
  linkSubtitle: { fontSize: 13, color: colors.grayMedium },
  viewIndicator: {
    width: 32,
    height: 32,
    borderRadius: 8,
    backgroundColor: colors.grayLight,
    alignItems: "center",
    justifyContent: "center",
  },
  tipTitle: { fontSize: 15, fontWeight: "600", color: colors.navy },
  tipText: { fontSize: 13, color: colors.grayMedium },
});

export default PrivacyAndSafety;
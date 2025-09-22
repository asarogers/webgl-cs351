import React, { useState } from "react";
import {
  View,
  Text,
  FlatList,
  TouchableOpacity,
  StyleSheet,
  Alert,
} from "react-native";
import {
  Ionicons,
  MaterialIcons,
  MaterialCommunityIcons,
  FontAwesome5,
} from "@expo/vector-icons";
import { SafeAreaView } from 'react-native-safe-area-context';

import ProfileManagement from "./1_ProfileManagement";
import { Notifications } from "./2_Notification";
import { ApplicationManager } from "./3_ApplicationManager";
import PrivacyAndSafety from "./4_Privacy_Safety";
import FeedbackAndAbout from "./5_FeedbackAndAbout";
import SupportScreen from "./6_Support";

// Settings data organized into sections using brand colors
const SETTINGS_SECTIONS = [
  {
    title: "Account & Preferences",
    titleColor: "#3B82F6",
    data: [
      {
        key: "profile",
        label: "Account",
        subtitle: "Manage your profile and personal information",
        icon: <Ionicons name="person-outline" size={24} color="#3B82F6" />,
        bg: "linear-gradient(135deg, #DBEAFE 0%, #EFF6FF 100%)",
        bgSolid: "#DBEAFE",
        iconBg: "#3B82F6",
      },
      {
        key: "notifications",
        label: "Notifications",
        subtitle: "Customize your notification preferences",
        icon: (
          <Ionicons name="notifications-outline" size={24} color="#2563EB" />
        ),
        bg: "linear-gradient(135deg, #BFDBFE 0%, #DBEAFE 100%)",
        bgSolid: "#BFDBFE",
        iconBg: "#2563EB",
      },
    ],
  },
  {
    title: "Applications",
    titleColor: "#F59E42", // Unique amber
    data: [
      {
        key: "applications",
        label: "Applications",
        subtitle: "Manage your apartment and roommate applications",
        icon: <FontAwesome5 name="file-alt" size={22} color="#F59E42" />,
        bgSolid: "#FEF3C7", // Soft light amber background
        iconBg: "#F59E42",
      },
    ],
  },

  {
    title: "Legal & Safety",
    titleColor: "#64748B",
    data: [
      {
        key: "privacy",
        label: "Privacy and Safety",
        subtitle: "Control your privacy and security settings",
        icon: (
          <MaterialCommunityIcons
            name="shield-account-outline"
            size={24}
            color="#334155"
          />
        ),
        bg: "linear-gradient(135deg, #CBD5E1 0%, #E2E8F0 100%)",
        bgSolid: "#CBD5E1",
        iconBg: "#334155",
      },
    ],
  },
  {
    title: "Help & Support",
    titleColor: "#10B981",
    data: [
      {
        key: "feedback",
        label: "Feedback and About",
        subtitle: "Send feedback and learn about the app",
        icon: <MaterialIcons name="feedback" size={24} color="#059669" />,
        bg: "linear-gradient(135deg, #A7F3D0 0%, #D1FAE5 100%)",
        bgSolid: "#A7F3D0",
        iconBg: "#059669",
      },
      {
        key: "support",
        label: "Support",
        subtitle: "Contact support for help",
        icon: <MaterialIcons name="support-agent" size={24} color="#10B981" />,
        bgSolid: "#A7F3D0",
        iconBg: "#10B981",
      },
    ],
  },
];

const SettingsScreen = () => {
  const [page, setPage] = useState("home");

  const handleSignOut = () => {
    Alert.alert(
      "Sign Out",
      "Are you sure you want to sign out of your account?",
      [
        {
          text: "Cancel",
          style: "cancel",
        },
        {
          text: "Sign Out",
          style: "destructive",
          onPress: () => {
            // Add your sign out logic here
            console.log("User signed out");
            // Example: navigation.reset({ index: 0, routes: [{ name: 'Login' }] });
          },
        },
      ]
    );
  };

  const renderSettingItem = ({ item }) => (
    <TouchableOpacity
      style={[styles.card, { backgroundColor: item.bgSolid }]}
      onPress={() => setPage(item.key)}
      activeOpacity={0.7}
    >
      <View style={[styles.iconWrapper, { backgroundColor: item.iconBg }]}>
        <View style={styles.iconInner}>{item.icon}</View>
      </View>
      <View style={styles.textContainer}>
        <Text style={styles.label}>{item.label}</Text>
        <Text style={styles.subtitle}>{item.subtitle}</Text>
      </View>
      <Ionicons
        name="chevron-forward"
        size={20}
        color="#9CA3AF"
        style={styles.arrow}
      />
    </TouchableOpacity>
  );

  const renderSectionHeader = ({ section }) => (
    <View style={styles.sectionHeaderContainer}>
      <View
        style={[
          styles.sectionTitleDot,
          { backgroundColor: section.titleColor },
        ]}
      />
      <Text style={[styles.sectionHeaderText, { color: section.titleColor }]}>
        {section.title}
      </Text>
    </View>
  );

  // Main list page with improved layout
  const renderHome = () => (
    <SafeAreaView style={styles.container}>
      <View style={styles.header}>
        <View style={styles.headerContent}>
          <View style={styles.headerIconContainer}>
            <Ionicons name="settings-sharp" size={24} color="#fff" />
          </View>
          <View>
            <Text style={styles.headerTitle}>Settings</Text>
            <Text style={styles.headerSubtitle}>
              Manage your account and preferences
            </Text>
          </View>
        </View>
      </View>

      <View style={styles.contentContainer}>
        <FlatList
          data={SETTINGS_SECTIONS}
          renderItem={({ item: section }) => (
            <View style={styles.section}>
              {renderSectionHeader({ section })}
              {section.data.map((item) => (
                <React.Fragment key={item.key}>
                  {renderSettingItem({ item })}
                </React.Fragment>
              ))}
            </View>
          )}
          keyExtractor={(item, index) => `section-${index}`}
          contentContainerStyle={styles.listContent}
          showsVerticalScrollIndicator={false}
        />

        {/* Sign Out Button */}
        <View style={styles.signOutContainer}>
          <TouchableOpacity
            style={styles.signOutButton}
            onPress={handleSignOut}
            activeOpacity={0.8}
          >
            <MaterialCommunityIcons
              name="logout"
              size={22}
              color="#DC2626"
              style={styles.signOutIcon}
            />
            <Text style={styles.signOutText}>Sign Out</Text>
          </TouchableOpacity>
        </View>
      </View>
    </SafeAreaView>
  );

  switch (page) {
    case "home":
      return renderHome();
    case "profile":
      return <ProfileManagement setPage={setPage} />;
    case "notifications":
      return <Notifications setPage={setPage} />;
    case "applications":
      return <ApplicationManager setPage={setPage} />;
    case "privacy":
      return <PrivacyAndSafety setPage={setPage} />;
    case "feedback": // FeedbackAndAbout
      return <FeedbackAndAbout setPage={setPage} />;
    case "support":
      return <SupportScreen setPage={setPage} />;
    default:
      return renderHome();
  }
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#F8FAFC",
  },
  header: {
    backgroundColor: "#1F2937",
    paddingBottom: 24,
    borderBottomLeftRadius: 0,
    borderBottomRightRadius: 0,
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
    paddingTop: 16,
  },
  headerIconContainer: {
    width: 48,
    height: 48,
    borderRadius: 24,
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    justifyContent: "center",
    alignItems: "center",
    marginRight: 16,
  },
  headerTitle: {
    color: "#fff",
    fontSize: 24,
    fontWeight: "700",
    marginBottom: 4,
  },
  headerSubtitle: {
    color: "rgba(255, 255, 255, 0.8)",
    fontSize: 14,
    fontWeight: "400",
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
  contentContainer: {
    flex: 1,
    paddingTop: 20,
  },
  listContent: {
    paddingBottom: 20,
  },
  section: {
    marginBottom: 24,
  },
  sectionHeaderContainer: {
    flexDirection: "row",
    alignItems: "center",
    paddingHorizontal: 20,
    paddingBottom: 12,
  },
  sectionTitleDot: {
    width: 4,
    height: 16,
    borderRadius: 2,
    marginRight: 12,
  },
  sectionHeaderText: {
    fontSize: 12,
    fontWeight: "700",
    textTransform: "uppercase",
    letterSpacing: 0.8,
  },
  card: {
    flexDirection: "row",
    alignItems: "center",
    padding: 16,
    marginHorizontal: 20,
    marginBottom: 8,
    borderRadius: 16,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 3 },
    shadowOpacity: 0.08,
    shadowRadius: 12,
    elevation: 4,
  },
  iconWrapper: {
    width: 48,
    height: 48,
    borderRadius: 14,
    justifyContent: "center",
    alignItems: "center",
    marginRight: 16,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.15,
    shadowRadius: 4,
    elevation: 3,
  },
  iconInner: {
    width: 32,
    height: 32,
    borderRadius: 8,
    backgroundColor: "rgba(255, 255, 255, 0.9)",
    justifyContent: "center",
    alignItems: "center",
  },
  textContainer: {
    flex: 1,
  },
  label: {
    fontSize: 16,
    fontWeight: "600",
    color: "#1F2937",
    marginBottom: 2,
  },
  subtitle: {
    fontSize: 13,
    color: "#6B7280",
    lineHeight: 18,
  },
  arrow: {
    marginLeft: 12,
    opacity: 0.6,
  },

  // Sign Out Styles
  signOutContainer: {
    paddingHorizontal: 20,
    paddingVertical: 16,
    borderTopWidth: 1,
    borderTopColor: "#E5E7EB",
    backgroundColor: "#fff",
  },
  signOutButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    backgroundColor: "#FEF2F2",
    paddingVertical: 16,
    paddingHorizontal: 24,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: "#FECACA",
  },
  signOutIcon: {
    marginRight: 8,
  },
  signOutText: {
    fontSize: 16,
    fontWeight: "600",
    color: "#DC2626",
  },

  // Placeholder screen styles
  placeholderContent: {
    flex: 1,
    justifyContent: "center",
    alignItems: "center",
    paddingHorizontal: 40,
  },
  placeholderIcon: {
    marginBottom: 16,
  },
  placeholderTitle: {
    fontSize: 20,
    fontWeight: "600",
    color: "#1F2937",
    marginBottom: 8,
  },
  placeholderText: {
    fontSize: 14,
    color: "#6B7280",
    textAlign: "center",
    lineHeight: 20,
  },
});

export default SettingsScreen;

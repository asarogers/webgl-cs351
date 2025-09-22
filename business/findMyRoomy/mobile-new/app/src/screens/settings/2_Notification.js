import React, { useState } from "react";
import {
  View,
  Text,
  ScrollView,
  TouchableOpacity,
  Switch,
  StyleSheet,

} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import {
  Ionicons,
  MaterialIcons,
  MaterialCommunityIcons,
  FontAwesome5,
} from "@expo/vector-icons";
import SettingsHeader from "./SettingsHeader";
export default function Notifications({ setPage }){
  const [notifications, setNotifications] = useState({
    // Profile & Discovery Notifications
    profileViews: true,
    profileLikes: true,
    newMatches: true,
    compatibilityUpdates: false,
    profileStrengthUpdates: true,
    verificationUpdates: true,

    // Messages & Communication (CometChat)
    newMessages: true,
    messageDelivery: false,
    messageRead: true,
    typingIndicators: true,
    groupMessages: true,
    groupMentions: true,
    incomingCalls: true,
    missedCalls: true,
    messageReactions: true,

    // Discovery & Search
    nearbyRoommates: true,
    budgetMatches: true,
    locationMatches: true,
    filterUpdates: false,
    savedSearchAlerts: true,
    dailyRecommendations: true,

    // Housing & Listings
    moveInReminders: true,
    budgetAlerts: true,
    newListings: true,
    priceDrops: true,
    availabilityUpdates: true,
    leaseExpiryReminders: true,

    // Map & Location
    mapNearbyUsers: false,
    locationBasedMatches: true,
    areaRecommendations: false,
    transportUpdates: false,
    safetyAlerts: true,

    // Social & Connections
    mutualConnections: true,
    friendRequests: true,
    connectionRequests: true,
    referralRewards: false,
    communityUpdates: false,

    // Safety & Verification
    verificationReminders: true,
    identityVerification: true,
    backgroundCheckUpdates: true,
    reportUpdates: true,
    safetyTips: false,
    accountSecurity: true,

    // Activity & Engagement
    profileBoosts: false,
    engagementTips: false,
    weeklyStats: false,
    monthlyReports: false,
    anniversaryReminders: false,

    // System & App
    appUpdates: true,
    maintenanceAlerts: true,
    featureAnnouncements: false,
    systemMessages: true,

    // Preferences
    soundEnabled: true,
    vibrationEnabled: true,
    showPreviews: true,
    badgeCount: true,
    quietHours: false,
    doNotDisturb: false,
  });

  const toggleNotification = (key) => {
    setNotifications((prev) => ({
      ...prev,
      [key]: !prev[key],
    }));
  };

  const NotificationItem = ({
    icon,
    title,
    subtitle,
    notificationKey,
    iconColor = "#246BFD",
    iconBg = "#E6EEFF",
    badge = null,
  }) => (
    <View style={styles.optionCard}>
      <View style={[styles.iconWrapper, { backgroundColor: iconBg }]}>
        {icon}
        {badge && (
          <View style={styles.notificationBadge}>
            <Text style={styles.badgeText}>{badge}</Text>
          </View>
        )}
      </View>
      <View style={styles.optionText}>
        <Text style={styles.optionTitle}>{title}</Text>
        {subtitle && <Text style={styles.optionSubtitle}>{subtitle}</Text>}
      </View>
      <Switch
        value={notifications[notificationKey]}
        onValueChange={() => toggleNotification(notificationKey)}
        trackColor={{ false: "#E5E7EB", true: "#246BFD" }}
        thumbColor={notifications[notificationKey] ? "#fff" : "#fff"}
        ios_backgroundColor="#E5E7EB"
      />
    </View>
  );

  const SectionHeader = ({ title, icon, count = null }) => (
    <View style={styles.sectionHeader}>
      {icon}
      <Text style={styles.sectionTitle}>{title}</Text>
      {count && (
        <View style={styles.sectionBadge}>
          <Text style={styles.sectionBadgeText}>{count}</Text>
        </View>
      )}
    </View>
  );

  return (
    <SafeAreaView style={styles.container}>
      {/* Header */}
      <SettingsHeader
      title="Notifications"
      subtitle="Manage your notifications"
      onBack={() => setPage("home")} 
      />

      <ScrollView
        style={styles.scrollView}
        showsVerticalScrollIndicator={false}
      >

        {/* Profile & Discovery */}
        <View style={styles.section}>
          <Text style={styles.categoryTitle}>Profile & Discovery</Text>

          <NotificationItem
            icon={<Ionicons name="eye-outline" size={20} color="#246BFD" />}
            title="Profile Views"
            subtitle="When someone views your profile"
            notificationKey="profileViews"
            // badge="3"
          />

          <NotificationItem
            icon={
              <MaterialIcons
                name="favorite-outline"
                size={20}
                color="#EF4444"
              />
            }
            title="Profile Likes"
            subtitle="When someone likes your profile"
            notificationKey="profileLikes"
            iconColor="#EF4444"
            iconBg="#FEF2F2"
            // badge="7"
          />

          <NotificationItem
            icon={
              <MaterialCommunityIcons
                name="heart-multiple"
                size={20}
                color="#EC4899"
              />
            }
            title="New Matches"
            subtitle="High compatibility matches found"
            notificationKey="newMatches"
            iconColor="#EC4899"
            iconBg="#FDF2F8"
          />


        </View>

        {/* Messages & Communication */}
        <View style={styles.section}>
          <Text style={styles.categoryTitle}>Messages & Communication</Text>

          <NotificationItem
            icon={
              <MaterialCommunityIcons
                name="message-text"
                size={20}
                color="#246BFD"
              />
            }
            title="New Messages"
            subtitle="Direct messages and conversations"
            notificationKey="newMessages"
            // badge="12"
          />

          <NotificationItem
            icon={<MaterialIcons name="done-all" size={20} color="#10B981" />}
            title="Message Delivery"
            subtitle="Message delivery confirmations"
            notificationKey="messageDelivery"
            iconColor="#10B981"
            iconBg="#ECFDF5"
          />
        </View>

        {/* Discovery & Search */}
        <View style={styles.section}>
          <Text style={styles.categoryTitle}>Discovery & Search</Text>

          <NotificationItem
            icon={
              <MaterialCommunityIcons
                name="map-marker-radius"
                size={20}
                color="#06B6D4"
              />
            }
            title="Nearby Roommates"
            subtitle="New users in your area"
            notificationKey="nearbyRoommates"
            iconColor="#06B6D4"
            iconBg="#ECFEFF"
          />

          <NotificationItem
            icon={
              <MaterialCommunityIcons name="cash" size={20} color="#22C55E" />
            }
            title="Budget Matches"
            subtitle="Roommates within your budget range"
            notificationKey="budgetMatches"
            iconColor="#22C55E"
            iconBg="#ECFDF5"
          />

          <NotificationItem
            icon={
              <MaterialIcons name="location-on" size={20} color="#EF4444" />
            }
            title="Location Matches"
            subtitle="Perfect location matches found"
            notificationKey="locationMatches"
            iconColor="#EF4444"
            iconBg="#FEF2F2"
          />
        </View>

        {/* Safety & Verification */}
        <View style={styles.section}>
          <Text style={styles.categoryTitle}>Safety & Verification</Text>

          <NotificationItem
            icon={
              <MaterialCommunityIcons
                name="shield-check-outline"
                size={20}
                color="#8B5CF6"
              />
            }
            title="Verification Reminders"
            subtitle="Complete your verification steps"
            notificationKey="verificationReminders"
            iconColor="#8B5CF6"
            iconBg="#F3E8FF"
          />

          <NotificationItem
            icon={
              <MaterialCommunityIcons
                name="security"
                size={20}
                color="#DC2626"
              />
            }
            title="Account Security"
            subtitle="Login alerts and security updates"
            notificationKey="accountSecurity"
            iconColor="#DC2626"
            iconBg="#FEF2F2"
          />
        </View>

        {/* Notification Preferences */}
        <View style={styles.section}>
          <Text style={styles.categoryTitle}>Preferences</Text>

          <NotificationItem
            icon={<MaterialIcons name="volume-up" size={20} color="#246BFD" />}
            title="Sound"
            subtitle="Play notification sounds"
            notificationKey="soundEnabled"
          />

          <NotificationItem
            icon={
              <MaterialCommunityIcons
                name="vibrate"
                size={20}
                color="#8B5CF6"
              />
            }
            title="Vibration"
            subtitle="Vibrate for notifications"
            notificationKey="vibrationEnabled"
            iconColor="#8B5CF6"
            iconBg="#F3E8FF"
          />

          <NotificationItem
            icon={
              <MaterialCommunityIcons
                name="moon-waning-crescent"
                size={20}
                color="#6B7280"
              />
            }
            title="Quiet Hours"
            subtitle="Pause notifications during set hours"
            notificationKey="quietHours"
            iconColor="#6B7280"
            iconBg="#F9FAFB"
          />
        </View>
        <View style={styles.bottomSpacer} />
      </ScrollView>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#F5F8FD",
  },
  header: {
    backgroundColor: "#246BFD",
    paddingTop: 48,
    paddingBottom: 28,
    paddingHorizontal: 22,
    borderBottomLeftRadius: 18,
    borderBottomRightRadius: 18,
  },
  headerTitle: {
    color: "#fff",
    fontSize: 20,
    fontWeight: "bold",
    marginTop: 10,
  },
  headerSubtitle: {
    color: "#E3E8F1",
    fontSize: 13,
    marginTop: 2,
  },
  backRow: {
    flexDirection: "row",
    alignItems: "center",
    marginTop: 20,
    marginLeft: 18,
    marginBottom: 14,
  },
  backText: {
    color: "#246BFD",
    fontSize: 15,
    fontWeight: "500",
    marginLeft: 3,
  },
  scrollView: {
    marginTop: 20,
    flex: 1,
    paddingHorizontal: 18,
  },
  sectionHeader: {
    flexDirection: "row",
    alignItems: "center",
    marginLeft: 7,
    marginBottom: 16,
  },
  sectionTitle: {
    fontSize: 17,
    fontWeight: "600",
    color: "#191D21",
    marginLeft: 9,
    flex: 1,
  },
  sectionBadge: {
    backgroundColor: "#EF4444",
    borderRadius: 10,
    paddingHorizontal: 8,
    paddingVertical: 2,
    minWidth: 20,
    alignItems: "center",
  },
  sectionBadgeText: {
    color: "#FFFFFF",
    fontSize: 11,
    fontWeight: "700",
  },
  section: {
    marginBottom: 24,
  },
  categoryTitle: {
    fontSize: 14,
    fontWeight: "600",
    color: "#6B7280",
    marginBottom: 12,
    marginLeft: 4,
    textTransform: "uppercase",
    letterSpacing: 0.5,
  },
  optionCard: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#fff",
    borderRadius: 13,
    padding: 17,
    marginBottom: 10,
    shadowColor: "#000",
    shadowOpacity: 0.02,
    shadowRadius: 3,
    elevation: 1,
  },
  iconWrapper: {
    width: 36,
    height: 36,
    borderRadius: 9,
    justifyContent: "center",
    alignItems: "center",
    marginRight: 13,
    position: "relative",
  },
  notificationBadge: {
    position: "absolute",
    top: -4,
    right: -4,
    backgroundColor: "#EF4444",
    borderRadius: 8,
    paddingHorizontal: 5,
    paddingVertical: 1,
    minWidth: 16,
    alignItems: "center",
    borderWidth: 1,
    borderColor: "#FFFFFF",
  },
  badgeText: {
    color: "#FFFFFF",
    fontSize: 10,
    fontWeight: "700",
  },
  optionText: {
    flex: 1,
  },
  optionTitle: {
    fontSize: 15,
    fontWeight: "600",
    color: "#191D21",
  },
  optionSubtitle: {
    fontSize: 12,
    color: "#A1A5AE",
    marginTop: 2,
  },
  actionButton: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#fff",
    borderRadius: 13,
    padding: 17,
    marginBottom: 10,
    shadowColor: "#000",
    shadowOpacity: 0.02,
    shadowRadius: 3,
    elevation: 1,
  },
  actionButtonText: {
    flex: 1,
    fontSize: 15,
    fontWeight: "500",
    color: "#191D21",
    marginLeft: 13,
  },
  bottomSpacer: {
    height: 20,
  },
});

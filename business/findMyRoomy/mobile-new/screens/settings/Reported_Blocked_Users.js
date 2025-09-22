// BlockedReportedUsersScreen.js
import React, { useEffect, useState } from "react";
import {
  View,
  Text,
  ScrollView,
  StyleSheet,
  ActivityIndicator,
  TouchableOpacity,
  Alert,
  RefreshControl,
} from "react-native";
import SettingsHeader from "./SettingsHeader";
import { Ionicons } from "@expo/vector-icons";
import authService from "../../database/authService";
import { SafeAreaView } from 'react-native-safe-area-context';

const colors = {
  primary: "#3B82F6",
  navy: "#1F2937",
  slate: "#374151",
  grayMedium: "#6B7280",
  grayLight: "#E5E7EB",
  background: "#F8FAFC",
  card: "#FFFFFF",
  error: "#EF4444",
  warning: "#F59E0B",
  success: "#10B981",
};

export default function BlockedReportedUsersScreen({ onBack }) {
  const [blockedUsers, setBlockedUsers] = useState([]);
  const [reportedUsers, setReportedUsers] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [processingFlags, setProcessingFlags] = useState(new Set());

  const fetchFlags = async (showRefreshIndicator = false) => {
    if (showRefreshIndicator) {
      setIsRefreshing(true);
    } else {
      setIsLoading(true);
    }
    
    try {
      const flags = await authService.getUserFlags();
      if (Array.isArray(flags)) {
        setBlockedUsers(flags.filter((f) => f.flag_type === "block"));
        setReportedUsers(flags.filter((f) => f.flag_type === "report"));
      }
    } catch (error) {
      console.error("Error fetching flags:", error);
      Alert.alert("Error", "Failed to load blocked and reported users");
    } finally {
      setIsLoading(false);
      setIsRefreshing(false);
    }
  };

  useEffect(() => {
    fetchFlags();
  }, []);

  const handleUnblock = (user) => {
    Alert.alert(
      "Unblock User",
      `Are you sure you want to unblock ${user.flagged_user?.first_name || "this user"}?`,
      [
        { text: "Cancel", style: "cancel" },
        {
          text: "Unblock",
          style: "default",
          onPress: () => removeFlag(user, "unblocked"),
        },
      ]
    );
  };

  const handleRemoveReport = (user) => {
    Alert.alert(
      "Remove Report",
      `Remove your report against ${user.flagged_user?.first_name || "this user"}?`,
      [
        { text: "Cancel", style: "cancel" },
        {
          text: "Remove",
          style: "default",
          onPress: () => removeFlag(user, "report removed"),
        },
      ]
    );
  };

  const removeFlag = async (flagItem, actionText) => {
    const flagId = flagItem.id;
    setProcessingFlags(prev => new Set(prev).add(flagId));

    try {
      // You'll need to implement this in your authService
      await authService.removeUserFlag(flagId);
      
      // Update local state immediately for better UX
      if (flagItem.flag_type === "block") {
        setBlockedUsers(prev => prev.filter(item => item.id !== flagId));
      } else {
        setReportedUsers(prev => prev.filter(item => item.id !== flagId));
      }

      // Show success message
      Alert.alert("Success", `User ${actionText} successfully`);
    } catch (error) {
      console.error("Error removing flag:", error);
      Alert.alert("Error", `Failed to remove flag. Please try again.`);
    } finally {
      setProcessingFlags(prev => {
        const newSet = new Set(prev);
        newSet.delete(flagId);
        return newSet;
      });
    }
  };

  const UserCard = ({ item, type, onAction, actionText, actionIcon, actionColor }) => {
    const isProcessing = processingFlags.has(item.id);
    const user = item.flagged_user;
    const displayName = user ? `${user.first_name || ""} ${user.last_name || ""}`.trim() : "Unknown User";
    const displayEmail = user?.email || "(user deleted)";

    return (
      <View style={styles.userCard}>
        <View style={styles.cardHeader}>
          <View style={styles.userInfo}>
            <Ionicons
              name={type === "block" ? "remove-circle-outline" : "alert-circle-outline"}
              size={20}
              color={type === "block" ? colors.error : colors.warning}
              style={styles.typeIcon}
            />
            <View style={styles.userDetails}>
              <Text style={styles.userName}>{displayName}</Text>
              <Text style={styles.userEmail}>{displayEmail}</Text>
              {item.reason && (
                <Text style={styles.userReason}>Reason: {item.reason}</Text>
              )}
              <Text style={styles.flagDate}>
                {type === "block" ? "Blocked" : "Reported"} on{" "}
                {new Date(item.created_at).toLocaleDateString()}
              </Text>
            </View>
          </View>
          
          <TouchableOpacity
            style={[
              styles.actionButton,
              { backgroundColor: actionColor + "15" },
              isProcessing && styles.actionButtonDisabled
            ]}
            onPress={() => onAction(item)}
            disabled={isProcessing}
          >
            {isProcessing ? (
              <ActivityIndicator size="small" color={actionColor} />
            ) : (
              <>
                <Ionicons name={actionIcon} size={16} color={actionColor} />
                <Text style={[styles.actionButtonText, { color: actionColor }]}>
                  {actionText}
                </Text>
              </>
            )}
          </TouchableOpacity>
        </View>
      </View>
    );
  };

  const EmptyState = ({ type }) => (
    <View style={styles.emptyState}>
      <Ionicons
        name={type === "blocked" ? "shield-checkmark-outline" : "checkmark-circle-outline"}
        size={48}
        color={colors.grayMedium}
      />
      <Text style={styles.emptyTitle}>
        No {type} users
      </Text>
      <Text style={styles.emptyText}>
        {type === "blocked" 
          ? "You haven't blocked anyone yet. Users you block will appear here."
          : "You haven't reported anyone yet. Users you report will appear here."
        }
      </Text>
    </View>
  );

  return (
    <SafeAreaView style={styles.container}>
      <SettingsHeader title="Blocked & Reported Users" onBack={onBack} />
      
      <ScrollView 
        style={styles.scrollView}
        refreshControl={
          <RefreshControl
            refreshing={isRefreshing}
            onRefresh={() => fetchFlags(true)}
            colors={[colors.primary]}
          />
        }
      >
        {isLoading ? (
          <View style={styles.loadingContainer}>
            <ActivityIndicator size="large" color={colors.primary} />
            <Text style={styles.loadingText}>Loading...</Text>
          </View>
        ) : (
          <>
            {/* Blocked Users Section */}
            <View style={styles.section}>
              <View style={styles.sectionHeader}>
                <Text style={styles.sectionTitle}>Blocked Users</Text>
                {blockedUsers.length > 0 && (
                  <View style={styles.countBadge}>
                    <Text style={styles.countText}>{blockedUsers.length}</Text>
                  </View>
                )}
              </View>
              
              {blockedUsers.length === 0 ? (
                <EmptyState type="blocked" />
              ) : (
                blockedUsers.map((item) => (
                  <UserCard
                    key={item.id}
                    item={item}
                    type="block"
                    onAction={handleUnblock}
                    actionText="Unblock"
                    actionIcon="checkmark-circle-outline"
                    actionColor={colors.success}
                  />
                ))
              )}
            </View>

            {/* Reported Users Section */}
            <View style={styles.section}>
              <View style={styles.sectionHeader}>
                <Text style={styles.sectionTitle}>Reported Users</Text>
                {reportedUsers.length > 0 && (
                  <View style={styles.countBadge}>
                    <Text style={styles.countText}>{reportedUsers.length}</Text>
                  </View>
                )}
              </View>
              
              {reportedUsers.length === 0 ? (
                <EmptyState type="reported" />
              ) : (
                reportedUsers.map((item) => (
                  <UserCard
                    key={item.id}
                    item={item}
                    type="report"
                    onAction={handleRemoveReport}
                    actionText="Remove"
                    actionIcon="close-circle-outline"
                    actionColor={colors.primary}
                  />
                ))
              )}
            </View>

            {/* Info Section */}
            <View style={styles.infoSection}>
              <Ionicons name="information-circle-outline" size={20} color={colors.grayMedium} />
              <Text style={styles.infoText}>
                Blocked users won't be able to contact you or see your profile. 
                Reported users are flagged for review by our moderation team.
              </Text>
            </View>
          </>
        )}
      </ScrollView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: colors.background,
  },
  scrollView: {
    flex: 1,
    padding: 18,
  },
  loadingContainer: {
    alignItems: "center",
    justifyContent: "center",
    paddingTop: 60,
  },
  loadingText: {
    marginTop: 12,
    fontSize: 14,
    color: colors.grayMedium,
  },
  section: {
    marginBottom: 32,
  },
  sectionHeader: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 16,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: "700",
    color: colors.slate,
    letterSpacing: 0.3,
  },
  countBadge: {
    backgroundColor: colors.primary,
    borderRadius: 12,
    paddingHorizontal: 8,
    paddingVertical: 2,
    marginLeft: 8,
  },
  countText: {
    color: "white",
    fontSize: 12,
    fontWeight: "600",
  },
  userCard: {
    backgroundColor: colors.card,
    borderRadius: 12,
    marginBottom: 12,
    padding: 16,
    borderWidth: 1,
    borderColor: colors.grayLight,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.05,
    shadowRadius: 2,
    elevation: 1,
  },
  cardHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "flex-start",
  },
  userInfo: {
    flexDirection: "row",
    flex: 1,
    marginRight: 12,
  },
  typeIcon: {
    marginRight: 12,
    marginTop: 2,
  },
  userDetails: {
    flex: 1,
  },
  userName: {
    fontSize: 16,
    color: colors.navy,
    fontWeight: "600",
    marginBottom: 2,
  },
  userEmail: {
    fontSize: 14,
    color: colors.grayMedium,
    marginBottom: 4,
  },
  userReason: {
    fontSize: 13,
    color: colors.warning,
    marginBottom: 4,
    fontStyle: "italic",
  },
  flagDate: {
    fontSize: 12,
    color: colors.grayMedium,
  },
  actionButton: {
    flexDirection: "row",
    alignItems: "center",
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 8,
    minWidth: 80,
    justifyContent: "center",
  },
  actionButtonDisabled: {
    opacity: 0.6,
  },
  actionButtonText: {
    fontSize: 13,
    fontWeight: "600",
    marginLeft: 4,
  },
  emptyState: {
    alignItems: "center",
    paddingVertical: 40,
    paddingHorizontal: 20,
  },
  emptyTitle: {
    fontSize: 18,
    fontWeight: "600",
    color: colors.slate,
    marginTop: 16,
    marginBottom: 8,
  },
  emptyText: {
    fontSize: 14,
    color: colors.grayMedium,
    textAlign: "center",
    lineHeight: 20,
  },
  infoSection: {
    flexDirection: "row",
    backgroundColor: colors.primary + "08",
    padding: 16,
    borderRadius: 12,
    marginTop: 8,
  },
  infoText: {
    flex: 1,
    fontSize: 13,
    color: colors.grayMedium,
    lineHeight: 18,
    marginLeft: 8,
  },
});
import React, { useState } from "react";
import {
  View,
  Text,
  ScrollView,
  TouchableOpacity,
  StyleSheet,
  Alert
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import {
  Ionicons,
  MaterialIcons,
  MaterialCommunityIcons,
  FontAwesome5,
} from "@expo/vector-icons";
import SettingsHeader from "./SettingsHeader";

// Design System Colors
const colors = {
  primary: {
    blue: '#3B82F6',
    blueHover: '#2563EB',
  },
  neutral: {
    navy: '#1F2937',
    slate: '#374151',
    grayMedium: '#6B7280',
    grayLight: '#E5E7EB',
    background: '#F8FAFC',
  },
  semantic: {
    success: '#10B981',
    warning: '#F59E0B',
    error: '#EF4444',
    info: '#3B82F6',
  }
};

const APPLICATION_STATUS_COLORS = {
  draft: "#6B7280",           // Gray - still working on it
  incomplete: "#F59E0B",      // Orange - missing info
  waitingForOthers: "#3B82F6", // Blue - waiting for roommates
  readyToSubmit: "#8B5CF6",   // Purple - ready but not sent
  submitted: "#10B981",       // Green - successfully sent
  withdrawn: "#64748B",       // Slate - user cancelled
};

const APPLICATION_STATUS_BG = {
  draft: "#F3F4F6",
  incomplete: "#FEF3C7",
  waitingForOthers: "#EBF4FF",
  readyToSubmit: "#F3E8FF",
  submitted: "#D1FAE5",
  withdrawn: "#F1F5F9",
};

// More realistic sample applications
const SAMPLE_APPLICATIONS = [
  {
    id: "1",
    title: "2BR Apartment in Williamsburg",
    createdAt: "2025-07-15",
    lastUpdated: "2025-07-20",
    status: "waitingForOthers",
    location: "Brooklyn, NY",
    landlord: "Madison Rentals",
    rent: "$3,200/month",
    applicants: [
      { name: "You", completed: true, avatar: null },
      { name: "Sarah Chen", completed: true, avatar: "SC" },
      { name: "Mike Rodriguez", completed: false, avatar: "MR" }
    ],
    missingDocuments: [],
    completionPercentage: 67,
  },
  {
    id: "2",
    title: "Loft - Midtown",
    createdAt: "2025-07-02",
    lastUpdated: "2025-07-18",
    status: "submitted",
    location: "Manhattan, NY",
    landlord: "Urban Spaces",
    rent: "$4,800/month",
    applicants: [
      { name: "You", completed: true, avatar: null },
      { name: "Alex Kim", completed: true, avatar: "AK" }
    ],
    submittedAt: "2025-07-18",
    completionPercentage: 100,
  },
  {
    id: "3",
    title: "1BR - Hoboken Waterfront",
    createdAt: "2025-06-18",
    lastUpdated: "2025-07-10",
    status: "incomplete",
    location: "Hoboken, NJ",
    landlord: "Gold Rentals",
    rent: "$2,800/month",
    applicants: [
      { name: "You", completed: false, avatar: null }
    ],
    missingDocuments: ["Pay stubs", "Bank statements", "References"],
    completionPercentage: 35,
  },
  {
    id: "4",
    title: "Studio - Long Island City",
    createdAt: "2025-07-10",
    lastUpdated: "2025-07-15",
    status: "readyToSubmit",
    location: "Queens, NY",
    landlord: "LIC Group",
    rent: "$2,100/month",
    applicants: [
      { name: "You", completed: true, avatar: null }
    ],
    missingDocuments: [],
    completionPercentage: 100,
  },
  {
    id: "5",
    title: "3BR Brownstone - Park Slope",
    createdAt: "2025-07-01",
    lastUpdated: "2025-07-05",
    status: "draft",
    location: "Brooklyn, NY",
    landlord: "Brooklyn Heights Realty",
    rent: "$5,500/month",
    applicants: [
      { name: "You", completed: false, avatar: null },
      { name: "Emma Johnson", completed: false, avatar: "EJ" },
      { name: "David Park", completed: false, avatar: "DP" }
    ],
    missingDocuments: ["Employment verification", "Credit report"],
    completionPercentage: 15,
  },
];

const statusInfo = {
  draft: {
    label: "Draft",
    description: "Application started but not complete",
    icon: "document-outline",
    actionText: "Continue"
  },
  incomplete: {
    label: "Incomplete",
    description: "Missing required information",
    icon: "warning-outline",
    actionText: "Complete"
  },
  waitingForOthers: {
    label: "Waiting for Others",
    description: "Roommates need to complete their sections",
    icon: "people-outline",
    actionText: "Remind"
  },
  readyToSubmit: {
    label: "Ready to Submit",
    description: "All information complete, ready to send",
    icon: "checkmark-circle-outline",
    actionText: "Submit"
  },
  submitted: {
    label: "Submitted",
    description: "Application successfully sent to landlord",
    icon: "paper-plane-outline",
    actionText: "View"
  },
  withdrawn: {
    label: "Withdrawn",
    description: "Application cancelled",
    icon: "close-circle-outline",
    actionText: null
  }
};

const ApplicantAvatars = ({ applicants }) => (
  <View style={styles.applicantsRow}>
    <Text style={styles.applicantsLabel}>Applicants:</Text>
    <View style={styles.avatarsContainer}>
      {applicants.map((applicant, index) => (
        <View key={index} style={styles.avatarContainer}>
          <View style={[
            styles.avatar,
            { backgroundColor: applicant.completed ? colors.semantic.success : colors.semantic.warning }
          ]}>
            <Text style={styles.avatarText}>
              {applicant.avatar || "Y"}
            </Text>
          </View>
          <View style={[
            styles.completionDot,
            { backgroundColor: applicant.completed ? colors.semantic.success : colors.semantic.warning }
          ]} />
        </View>
      ))}
    </View>
  </View>
);

const ProgressBar = ({ percentage, status }) => (
  <View style={styles.progressContainer}>
    <View style={styles.progressBG}>
      <View style={[
        styles.progressFill,
        {
          width: `${percentage}%`,
          backgroundColor: APPLICATION_STATUS_COLORS[status]
        }
      ]} />
    </View>
    <Text style={styles.progressText}>{percentage}% complete</Text>
  </View>
);

const ApplicationCard = ({ application, onAction }) => {
  const status = statusInfo[application.status];
  
  const handleAction = () => {
    switch (application.status) {
      case 'draft':
      case 'incomplete':
        onAction('continue', application.id);
        break;
      case 'waitingForOthers':
        onAction('remind', application.id);
        break;
      case 'readyToSubmit':
        onAction('submit', application.id);
        break;
      case 'submitted':
        onAction('view', application.id);
        break;
    }
  };

  return (
    <View style={[
      styles.applicationCard,
      { backgroundColor: APPLICATION_STATUS_BG[application.status] }
    ]}>
      <View style={styles.cardHeader}>
        <Ionicons
          name={status.icon}
          size={24}
          color={APPLICATION_STATUS_COLORS[application.status]}
          style={styles.cardIcon}
        />
        <View style={{ flex: 1 }}>
          <Text style={styles.appTitle}>{application.title}</Text>
          <Text style={styles.appLocation}>{application.location}</Text>
          <Text style={styles.appRent}>{application.rent}</Text>
        </View>
        <View
          style={[
            styles.statusBadge,
            {
              backgroundColor: APPLICATION_STATUS_COLORS[application.status] + "20",
              borderColor: APPLICATION_STATUS_COLORS[application.status],
            },
          ]}
        >
          <Text style={[
            styles.statusText,
            { color: APPLICATION_STATUS_COLORS[application.status] }
          ]}>
            {status.label}
          </Text>
        </View>
      </View>

      {/* Progress Bar */}
      <ProgressBar percentage={application.completionPercentage} status={application.status} />

      {/* Applicants */}
      {application.applicants.length > 1 && (
        <ApplicantAvatars applicants={application.applicants} />
      )}

      {/* Missing Documents */}
      {application.missingDocuments && application.missingDocuments.length > 0 && (
        <View style={styles.missingDocs}>
          <MaterialIcons name="error-outline" size={16} color={colors.semantic.warning} />
          <Text style={styles.missingDocsText}>
            Missing: {application.missingDocuments.join(", ")}
          </Text>
        </View>
      )}

      {/* Waiting for others details */}
      {application.status === 'waitingForOthers' && (
        <View style={styles.waitingDetails}>
          <MaterialCommunityIcons name="clock-outline" size={16} color={colors.primary.blue} />
          <Text style={styles.waitingText}>
            Waiting for {application.applicants.filter(a => !a.completed).map(a => a.name).join(", ")} to complete their sections
          </Text>
        </View>
      )}

      <View style={styles.cardFooter}>
        <Text style={styles.appDetail}>
          <Ionicons name="person-outline" size={14} color={colors.neutral.grayMedium} /> {application.landlord}
        </Text>
        {/* <Text style={styles.appDetail}>
          <Ionicons name="time-outline" size={14} color={colors.neutral.grayMedium} /> Updated {application.lastUpdated}
        </Text> */}
        <View style={{ flex: 1 }} />
        
        {status.actionText && (
          <TouchableOpacity
            style={[
              styles.actionBtn,
              { backgroundColor: APPLICATION_STATUS_COLORS[application.status] + "20" }
            ]}
            onPress={handleAction}
          >
            <Text style={[
              styles.actionText,
              { color: APPLICATION_STATUS_COLORS[application.status] }
            ]}>
              {status.actionText}
            </Text>
          </TouchableOpacity>
        )}

        {application.status !== 'withdrawn' && application.status !== 'submitted' && (
          <TouchableOpacity
            style={styles.withdrawBtn}
            onPress={() => onAction('withdraw', application.id)}
          >
            <MaterialCommunityIcons
              name="close-circle-outline"
              size={16}
              color={colors.semantic.error}
            />
          </TouchableOpacity>
        )}
      </View>
    </View>
  );
};

export const ApplicationManager = ({ setPage }) => {
  const [applications, setApplications] = useState(SAMPLE_APPLICATIONS);

  const handleAction = (action, id) => {
    switch (action) {
      case 'continue':
      case 'complete':
        Alert.alert("Continue Application", "Opening application form...");
        break;
      case 'remind':
        Alert.alert("Remind Roommates", "Sending reminder notifications to incomplete applicants...");
        break;
      case 'submit':
        Alert.alert(
          "Submit Application",
          "Are you sure you want to submit this application?",
          [
            { text: "Cancel", style: "cancel" },
            {
              text: "Submit",
              onPress: () => {
                setApplications(prev =>
                  prev.map(app =>
                    app.id === id
                      ? { ...app, status: "submitted", submittedAt: new Date().toISOString().split('T')[0] }
                      : app
                  )
                );
              }
            }
          ]
        );
        break;
      case 'view':
        Alert.alert("View Application", "Opening submitted application details...");
        break;
      case 'withdraw':
        Alert.alert(
          "Withdraw Application",
          "Are you sure you want to withdraw this application?",
          [
            { text: "Cancel", style: "cancel" },
            {
              text: "Withdraw",
              style: "destructive",
              onPress: () => {
                setApplications(prev =>
                  prev.map(app =>
                    app.id === id ? { ...app, status: "withdrawn" } : app
                  )
                );
              }
            }
          ]
        );
        break;
    }
  };

  const [selectedTab, setSelectedTab] = useState("active");
  const statusTabs = [
    { key: "active", label: "Active" },
    { key: "draft", label: "Draft" },
    { key: "incomplete", label: "Incomplete" },
    { key: "waitingForOthers", label: "Waiting" },
    { key: "readyToSubmit", label: "Ready" },
    { key: "submitted", label: "Submitted" },
    { key: "withdrawn", label: "Withdrawn" },
  ];

  const getFilteredApps = () => {
    if (selectedTab === "active") {
      return applications.filter(app => 
        !['submitted', 'withdrawn'].includes(app.status)
      );
    }
    return applications.filter(app => app.status === selectedTab);
  };

  const filteredApps = getFilteredApps();

  const getTabCount = (tabKey) => {
    if (tabKey === "active") {
      return applications.filter(app => 
        !['submitted', 'withdrawn'].includes(app.status)
      ).length;
    }
    return applications.filter(app => app.status === tabKey).length;
  };

  return (
    <SafeAreaView style={styles.container}>
      {/* Header */}
      <SettingsHeader
        title="Applications"
        subtitle="Track and manage your rental applications"
        onBack={() => setPage("home")}
      />

      {/* Quick Stats */}
      <View style={styles.statsContainer}>
        <View style={styles.statCard}>
          <Text style={styles.statNumber}>{getTabCount("active")}</Text>
          <Text style={styles.statLabel}>Active</Text>
        </View>
        <View style={styles.statCard}>
          <Text style={styles.statNumber}>{getTabCount("submitted")}</Text>
          <Text style={styles.statLabel}>Submitted</Text>
        </View>
        <View style={styles.statCard}>
          <Text style={styles.statNumber}>{getTabCount("waitingForOthers")}</Text>
          <Text style={styles.statLabel}>Waiting</Text>
        </View>
        <View style={styles.statCard}>
          <Text style={styles.statNumber}>{getTabCount("readyToSubmit")}</Text>
          <Text style={styles.statLabel}>Ready</Text>
        </View>
      </View>

      {/* Tabs */}
      <ScrollView 
        horizontal 
        showsHorizontalScrollIndicator={false}
        style={styles.tabsContainer}
      >
        <View style={styles.tabsRow}>
          {statusTabs.map((tab) => {
            const count = getTabCount(tab.key);
            return (
              <TouchableOpacity
                key={tab.key}
                style={[
                  styles.tabBtn,
                  selectedTab === tab.key && styles.tabBtnActive
                ]}
                onPress={() => setSelectedTab(tab.key)}
                activeOpacity={0.75}
              >
                <Text
                  style={[
                    styles.tabBtnText,
                    selectedTab === tab.key && styles.tabBtnTextActive
                  ]}
                >
                  {tab.label}
                  {count > 0 && ` (${count})`}
                </Text>
              </TouchableOpacity>
            );
          })}
        </View>
      </ScrollView>

      <ScrollView style={styles.scrollView}>
        {filteredApps.length === 0 ? (
          <View style={styles.emptyState}>
            <MaterialCommunityIcons
              name="file-document-outline"
              size={64}
              color={colors.neutral.grayLight}
            />
            <Text style={styles.emptyTitle}>No applications found</Text>
            <Text style={styles.emptyDesc}>
              {selectedTab === "active" 
                ? "You don't have any active applications. Start by browsing available listings."
                : `No applications in "${statusTabs.find(t => t.key === selectedTab)?.label}" status.`
              }
            </Text>
            {selectedTab === "active" && (
              <TouchableOpacity style={styles.browseBtn}>
                <Text style={styles.browseBtnText}>Browse Listings</Text>
              </TouchableOpacity>
            )}
          </View>
        ) : (
          filteredApps.map((app) => (
            <ApplicationCard
              key={app.id}
              application={app}
              onAction={handleAction}
            />
          ))
        )}
        <View style={{ height: 32 }} />
      </ScrollView>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: colors.neutral.background,
  },
  statsContainer: {
    flexDirection: 'row',
    paddingHorizontal: 18,
    paddingVertical: 16,
    gap: 12,
  },
  statCard: {
    flex: 1,
    backgroundColor: '#fff',
    borderRadius: 12,
    padding: 16,
    alignItems: 'center',
    shadowColor: "#000",
    shadowOpacity: 0.02,
    shadowRadius: 3,
    elevation: 1,
  },
  statNumber: {
    fontSize: 24,
    fontWeight: '800',
    color: colors.neutral.navy,
    marginBottom: 4,
  },
  statLabel: {
    fontSize: 12,
    color: colors.neutral.grayMedium,
    fontWeight: '600',
  },
  tabsContainer: {
    backgroundColor: '#fff',
    borderBottomColor: colors.neutral.grayLight,
    borderBottomWidth: 1,
    maxHeight: 60
  },
  tabsRow: {
    flexDirection: "row",
    paddingHorizontal: 18,
    paddingVertical: 8,
  },
  tabBtn: {
    marginRight: 12,
    paddingVertical: 6,
    paddingHorizontal: 12,
    borderRadius: 16,
    backgroundColor: "transparent",
    borderWidth: 1,
    borderColor: colors.neutral.grayLight,
  },
  tabBtnActive: {
    backgroundColor: colors.primary.blue,
    borderColor: colors.primary.blue,
  },
  tabBtnText: {
    fontSize: 14,
    color: colors.neutral.grayMedium,
    fontWeight: "600",
  },
  tabBtnTextActive: {
    color: "#fff",
    fontWeight: "700",
  },
  scrollView: {
    flex: 1,
    paddingHorizontal: 18,
    marginTop: 8,
  },
  applicationCard: {
    borderRadius: 16,
    padding: 20,
    marginBottom: 16,
    backgroundColor: "#fff",
    shadowColor: "#000",
    shadowOpacity: 0.03,
    shadowRadius: 6,
    shadowOffset: { width: 0, height: 2 },
    elevation: 2,
  },
  cardHeader: {
    flexDirection: "row",
    alignItems: "flex-start",
    marginBottom: 16,
  },
  cardIcon: {
    marginRight: 16,
    marginTop: 2,
  },
  appTitle: {
    fontSize: 18,
    fontWeight: "700",
    color: colors.neutral.navy,
    marginBottom: 4,
  },
  appLocation: {
    fontSize: 14,
    color: colors.neutral.grayMedium,
    marginBottom: 2,
  },
  appRent: {
    fontSize: 16,
    color: colors.semantic.success,
    fontWeight: "600",
  },
  statusBadge: {
    borderRadius: 12,
    borderWidth: 1,
    paddingHorizontal: 12,
    paddingVertical: 6,
    alignSelf: "flex-start",
    marginLeft: 12,
  },
  statusText: {
    fontWeight: "700",
    fontSize: 12,
    letterSpacing: 0.5,
    textTransform: "uppercase",
  },
  progressContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 12,
  },
  progressBG: {
    flex: 1,
    height: 8,
    backgroundColor: colors.neutral.grayLight,
    borderRadius: 4,
    marginRight: 12,
  },
  progressFill: {
    height: '100%',
    borderRadius: 4,
  },
  progressText: {
    fontSize: 12,
    fontWeight: '600',
    color: colors.neutral.grayMedium,
    minWidth: 80,
  },
  applicantsRow: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 12,
  },
  applicantsLabel: {
    fontSize: 12,
    color: colors.neutral.grayMedium,
    fontWeight: '600',
    marginRight: 12,
  },
  avatarsContainer: {
    flexDirection: 'row',
    gap: 8,
  },
  avatarContainer: {
    position: 'relative',
  },
  avatar: {
    width: 32,
    height: 32,
    borderRadius: 16,
    justifyContent: 'center',
    alignItems: 'center',
  },
  avatarText: {
    color: '#fff',
    fontSize: 12,
    fontWeight: '700',
  },
  completionDot: {
    position: 'absolute',
    bottom: -2,
    right: -2,
    width: 12,
    height: 12,
    borderRadius: 6,
    borderWidth: 2,
    borderColor: '#fff',
  },
  missingDocs: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    backgroundColor: '#FEF3C7',
    borderRadius: 8,
    padding: 12,
    marginBottom: 12,
  },
  missingDocsText: {
    flex: 1,
    fontSize: 12,
    color: '#92400E',
    marginLeft: 8,
    lineHeight: 16,
  },
  waitingDetails: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    backgroundColor: '#EBF4FF',
    borderRadius: 8,
    padding: 12,
    marginBottom: 12,
  },
  waitingText: {
    flex: 1,
    fontSize: 12,
    color: '#1E40AF',
    marginLeft: 8,
    lineHeight: 16,
  },
  cardFooter: {
    flexDirection: "row",
    alignItems: "center",
    marginTop: 8,
  },
  appDetail: {
    fontSize: 12,
    color: colors.neutral.grayMedium,
    marginRight: 16,
    flexDirection: "row",
    alignItems: "center",
  },
  actionBtn: {
    flexDirection: "row",
    alignItems: "center",
    borderRadius: 8,
    paddingHorizontal: 12,
    paddingVertical: 6,
    marginLeft: 8,
  },
  actionText: {
    fontWeight: "600",
    fontSize: 13,
  },
  withdrawBtn: {
    padding: 8,
    marginLeft: 8,
  },
  emptyState: {
    alignItems: "center",
    marginTop: 64,
    paddingHorizontal: 32,
  },
  emptyTitle: {
    fontSize: 20,
    fontWeight: "700",
    color: colors.neutral.grayMedium,
    marginTop: 20,
    marginBottom: 8,
  },
  emptyDesc: {
    color: colors.neutral.grayMedium,
    fontSize: 14,
    textAlign: "center",
    lineHeight: 20,
    marginBottom: 24,
  },
  browseBtn: {
    backgroundColor: colors.primary.blue,
    borderRadius: 12,
    paddingHorizontal: 24,
    paddingVertical: 12,
  },
  browseBtnText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
});

export default ApplicationManager;
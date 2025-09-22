import React, { useState } from "react";
import {
  View,
  Text,
  ScrollView,
  TouchableOpacity,
  StyleSheet,
  TextInput,
  Alert,
  Linking,
  KeyboardAvoidingView,
  Platform,
} from "react-native";

import { SafeAreaView } from 'react-native-safe-area-context';
import {
  Ionicons,
  MaterialIcons,
  MaterialCommunityIcons,
  Feather,
} from "@expo/vector-icons";
import SettingsHeader from "./SettingsHeader";

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
  success: "#10B981",
};

const CONTACT_INFO = {
  email: "softwareace.j@gmail.com",
  website: "https://findmyroomy.com",
  social: {
    twitter: "https://twitter.com/findmyroomy",
    instagram: "https://instagram.com/findmyroomy",
    linkedin: "https://linkedin.com/company/findmyroomy",
  },
};

const APP_INFO = {
  version: "2.4.1",
  buildNumber: "2024.07.001",
  company: "FindMyRoomy, Inc.",
  founded: "2022",
  team: "8+ people globally",
};

export const FeedbackAndAbout = ({ setPage }) => {
  const [activeTab, setActiveTab] = useState("feedback");
  const [feedbackType, setFeedbackType] = useState("general");
  const [feedbackText, setFeedbackText] = useState("");
  const [userEmail, setUserEmail] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);

  const feedbackTypes = [
    { id: "general", label: "General Feedback", icon: "chatbubble-outline", color: colors.primary },
    { id: "bug", label: "Report a Bug", icon: "bug-outline", color: colors.error },
    { id: "feature", label: "Feature Request", icon: "bulb-outline", color: colors.warning },
  ];

  const handleSubmitFeedback = async () => {
    if (!feedbackText.trim()) {
      Alert.alert("Missing Information", "Please enter your feedback before submitting.");
      return;
    }
    if (!userEmail.trim()) {
      Alert.alert("Missing Email", "Please enter your email so we can follow up with you.");
      return;
    }
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(userEmail)) {
      Alert.alert("Invalid Email", "Please enter a valid email address.");
      return;
    }
  
    // Compose subject and body
    const feedbackTypeLabel = feedbackTypes.find(t => t.id === feedbackType)?.label || "General Feedback";
    const subject = encodeURIComponent(`FindMyRoomy Feedback: ${feedbackTypeLabel}`);
    const body = encodeURIComponent(
      `Type: ${feedbackTypeLabel}\nEmail: ${userEmail}\n\nFeedback:\n${feedbackText}\n\n---\nSent from FindMyRoomy App`
    );
  
    // Popup: Choose email app
    Alert.alert(
      "Send With…",
      "Which email app would you like to use?",
      [
        {
          text: "Gmail",
          onPress: () => {
            let gmailUrl = "";
            if (Platform.OS === "android") {
              // mailto will open default, usually Gmail on Android
              gmailUrl = `mailto:${CONTACT_INFO.email}?subject=${subject}&body=${body}`;
            } else {
              // iOS: Try Gmail app if installed, else fallback to browser
              gmailUrl = `googlegmail://co?to=${CONTACT_INFO.email}&subject=${subject}&body=${body}`;
            }
            Linking.openURL(gmailUrl).catch(() => {
              // Fallback to browser Gmail compose
              const webGmailUrl = `https://mail.google.com/mail/?view=cm&fs=1&to=${CONTACT_INFO.email}&su=${subject}&body=${body}`;
              Linking.openURL(webGmailUrl).catch(() => {
                Alert.alert(
                  "Unable to open Gmail",
                  "Could not open Gmail. Please try Apple Mail or use the email address above."
                );
              });
            });
          },
        },
        {
          text: "Apple Mail",
          onPress: () => {
            const mailtoUrl = `mailto:${CONTACT_INFO.email}?subject=${subject}&body=${body}`;
            Linking.openURL(mailtoUrl).catch(() => {
              Alert.alert(
                "Unable to open Mail app",
                "Please email us at softwareace.j@gmail.com if the email app doesn't open."
              );
            });
          },
        },
        { text: "Cancel", style: "cancel" },
      ],
      { cancelable: true }
    );
  };
  

  const handleContactPress = (type, value) => {
    switch (type) {
      case "email":
        Linking.openURL(`mailto:${value}`);
        break;
      case "website":
      case "social":
        Linking.openURL(value);
        break;
      default:
        break;
    }
  };

  const renderTabButton = (tabId, label, icon) => (
    <TouchableOpacity
      style={[
        styles.tabButton,
        activeTab === tabId && styles.activeTabButton
      ]}
      onPress={() => setActiveTab(tabId)}
      activeOpacity={0.7}
    >
      <Ionicons 
        name={icon} 
        size={20} 
        color={activeTab === tabId ? colors.primary : colors.grayMedium} 
      />
      <Text style={[
        styles.tabLabel,
        activeTab === tabId && styles.activeTabLabel
      ]}>
        {label}
      </Text>
    </TouchableOpacity>
  );

  const renderFeedbackContent = () => (
    <KeyboardAvoidingView 
      behavior={Platform.OS === "ios" ? "padding" : "height"}
      style={{ flex: 1 }}
    >
      <ScrollView showsVerticalScrollIndicator={false}>
        <View style={styles.section}>
          <Text style={styles.sectionTitle}>What would you like to share?</Text>
          <View style={styles.feedbackTypeContainer}>
            {feedbackTypes.map((type) => (
              <TouchableOpacity
                key={type.id}
                style={[
                  styles.feedbackTypeCard,
                  feedbackType === type.id && styles.activeFeedbackType
                ]}
                onPress={() => setFeedbackType(type.id)}
                activeOpacity={0.7}
              >
                <View style={[
                  styles.typeIconContainer,
                  { backgroundColor: feedbackType === type.id ? `${type.color}15` : colors.grayLight }
                ]}>
                  <Ionicons 
                    name={type.icon} 
                    size={22} 
                    color={feedbackType === type.id ? type.color : colors.grayMedium} 
                  />
                </View>
                <Text style={[
                  styles.feedbackTypeLabel,
                  feedbackType === type.id && { color: type.color }
                ]}>
                  {type.label}
                </Text>
              </TouchableOpacity>
            ))}
          </View>
        </View>

        <View style={styles.section}>
          <Text style={styles.inputLabel}>Your Email</Text>
          <TextInput
            style={styles.emailInput}
            value={userEmail}
            onChangeText={setUserEmail}
            placeholder="your.email@example.com"
            placeholderTextColor={colors.grayMedium}
            keyboardType="email-address"
            autoCapitalize="none"
            autoCorrect={false}
          />
        </View>

        <View style={styles.section}>
          <Text style={styles.inputLabel}>
            {feedbackType === "bug" ? "Describe the bug" : 
             feedbackType === "feature" ? "Describe your feature idea" : 
             "Your feedback"}
          </Text>
          <TextInput
            style={styles.feedbackInput}
            value={feedbackText}
            onChangeText={setFeedbackText}
            placeholder={
              feedbackType === "bug" ? "Please describe what happened, what you expected, and steps to reproduce the issue..." :
              feedbackType === "feature" ? "Tell us about the feature you'd like to see and how it would help you..." :
              "Share your thoughts, suggestions, or any feedback you have about the app..."
            }
            placeholderTextColor={colors.grayMedium}
            multiline
            numberOfLines={6}
            textAlignVertical="top"
          />
        </View>

        <TouchableOpacity
          style={[
            styles.submitButton,
            (!feedbackText.trim() || !userEmail.trim() || isSubmitting) && styles.disabledButton
          ]}
          onPress={handleSubmitFeedback}
          disabled={!feedbackText.trim() || !userEmail.trim() || isSubmitting}
          activeOpacity={0.8}
        >
          {isSubmitting ? (
            <Text style={styles.submitButtonText}>Submitting...</Text>
          ) : (
            <>
              <Ionicons name="send" size={18} color="#FFFFFF" style={{ marginRight: 8 }} />
              <Text style={styles.submitButtonText}>Submit Feedback</Text>
            </>
          )}
        </TouchableOpacity>
      </ScrollView>
    </KeyboardAvoidingView>
  );

  const renderAboutContent = () => (
    <ScrollView showsVerticalScrollIndicator={false}>
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>App Information</Text>
        <View style={styles.infoCard}>
          <View style={styles.infoRow}>
            <Text style={styles.infoLabel}>Version</Text>
            <Text style={styles.infoValue}>{APP_INFO.version}</Text>
          </View>
          <View style={styles.infoRow}>
            <Text style={styles.infoLabel}>Build</Text>
            <Text style={styles.infoValue}>{APP_INFO.buildNumber}</Text>
          </View>
          <View style={styles.infoRow}>
            <Text style={styles.infoLabel}>Company</Text>
            <Text style={styles.infoValue}>{APP_INFO.company}</Text>
          </View>
        </View>
      </View>

      <View style={styles.section}>
        <Text style={styles.sectionTitle}>About Us</Text>
        <View style={styles.aboutCard}>
          <View style={[styles.iconContainer, { backgroundColor: '#EFF6FF' }]}>
            <MaterialCommunityIcons name="heart" size={24} color={colors.primary} />
          </View>
          <View style={styles.aboutContent}>
            <Text style={styles.aboutTitle}>Our Mission</Text>
            <Text style={styles.aboutText}>
              We're building the future of shared living. Founded in {APP_INFO.founded}, our team of {APP_INFO.team} is dedicated to creating meaningful connections and better homes for everyone.
            </Text>
          </View>
        </View>
      </View>

      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Get in Touch</Text>
        
        <TouchableOpacity
          style={styles.contactCard}
          onPress={() => handleContactPress("email", CONTACT_INFO.email)}
          activeOpacity={0.7}
        >
          <View style={[styles.iconContainer, { backgroundColor: '#FEF3F2' }]}>
            <MaterialIcons name="email" size={20} color={colors.error} />
          </View>
          <View style={styles.contentContainer}>
            <Text style={styles.contactLabel}>Email Support</Text>
            <Text style={styles.contactValue}>{CONTACT_INFO.email}</Text>
          </View>
          <Ionicons name="chevron-forward" size={18} color={colors.grayMedium} />
        </TouchableOpacity>

        <TouchableOpacity
          style={styles.contactCard}
          onPress={() => handleContactPress("website", CONTACT_INFO.website)}
          activeOpacity={0.7}
        >
          <View style={[styles.iconContainer, { backgroundColor: '#F0F9FF' }]}>
            <Feather name="globe" size={20} color={colors.primary} />
          </View>
          <View style={styles.contentContainer}>
            <Text style={styles.contactLabel}>Website</Text>
            <Text style={styles.contactValue}>findmyroomy.com</Text>
          </View>
          <Ionicons name="chevron-forward" size={18} color={colors.grayMedium} />
        </TouchableOpacity>
      </View>

      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Follow Us</Text>
        <View style={styles.socialContainer}>
          <TouchableOpacity
            style={styles.socialButton}
            onPress={() => handleContactPress("social", CONTACT_INFO.social.twitter)}
            activeOpacity={0.7}
          >
            <Ionicons name="logo-twitter" size={24} color="#1DA1F2" />
          </TouchableOpacity>
          <TouchableOpacity
            style={styles.socialButton}
            onPress={() => handleContactPress("social", CONTACT_INFO.social.instagram)}
            activeOpacity={0.7}
          >
            <Ionicons name="logo-instagram" size={24} color="#E4405F" />
          </TouchableOpacity>
          <TouchableOpacity
            style={styles.socialButton}
            onPress={() => handleContactPress("social", CONTACT_INFO.social.linkedin)}
            activeOpacity={0.7}
          >
            <Ionicons name="logo-linkedin" size={24} color="#0077B5" />
          </TouchableOpacity>
        </View>
      </View>

      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Legal</Text>
        <Text style={styles.legalText}>
          © {new Date().getFullYear()} {APP_INFO.company}. All rights reserved.
        </Text>
      </View>
    </ScrollView>
  );

  return (
    <SafeAreaView style={styles.container}>
      <SettingsHeader
        title="Feedback & About"
        subtitle="Share feedback or learn more about us"
        onBack={() => setPage("home")}
      />

      <View style={styles.tabContainer}>
        {renderTabButton("feedback", "Feedback", "chatbubble-outline")}
        {renderTabButton("about", "About", "information-circle-outline")}
      </View>

      <View style={styles.contentArea}>
        {activeTab === "feedback" ? renderFeedbackContent() : renderAboutContent()}
      </View>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: colors.background },
  tabContainer: {
    flexDirection: "row",
    backgroundColor: colors.card,
    marginHorizontal: 20,
    marginTop: 20,
    borderRadius: 12,
    padding: 4,
    shadowColor: "#000",
    shadowOpacity: 0.04,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 2 },
    elevation: 2,
  },
  tabButton: {
    flex: 1,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    paddingVertical: 12,
    paddingHorizontal: 16,
    borderRadius: 8,
  },
  activeTabButton: {
    backgroundColor: colors.background,
  },
  tabLabel: {
    fontSize: 14,
    fontWeight: "600",
    color: colors.grayMedium,
    marginLeft: 6,
  },
  activeTabLabel: {
    color: colors.primary,
  },
  contentArea: {
    flex: 1,
    paddingHorizontal: 20,
    paddingTop: 24,
  },
  section: {
    marginBottom: 28,
  },
  sectionTitle: {
    fontSize: 13,
    fontWeight: "700",
    color: colors.slate,
    textTransform: "uppercase",
    marginBottom: 16,
    letterSpacing: 0.8,
    marginLeft: 4,
  },
  feedbackTypeContainer: {
    flexDirection: "row",
    justifyContent: "space-between",
  },
  feedbackTypeCard: {
    flex: 1,
    alignItems: "center",
    backgroundColor: colors.card,
    borderRadius: 12,
    padding: 16,
    marginHorizontal: 4,
    borderWidth: 2,
    borderColor: "transparent",
  },
  activeFeedbackType: {
    borderColor: colors.primary,
  },
  typeIconContainer: {
    width: 48,
    height: 48,
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
    marginBottom: 8,
  },
  feedbackTypeLabel: {
    fontSize: 12,
    fontWeight: "600",
    color: colors.grayMedium,
    textAlign: "center",
  },
  inputLabel: {
    fontSize: 14,
    fontWeight: "600",
    color: colors.navy,
    marginBottom: 8,
    marginLeft: 4,
  },
  emailInput: {
    backgroundColor: colors.card,
    borderRadius: 12,
    padding: 16,
    fontSize: 16,
    color: colors.navy,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  feedbackInput: {
    backgroundColor: colors.card,
    borderRadius: 12,
    padding: 16,
    fontSize: 16,
    color: colors.navy,
    borderWidth: 1,
    borderColor: colors.grayLight,
    minHeight: 120,
  },
  submitButton: {
    backgroundColor: colors.primary,
    borderRadius: 12,
    padding: 16,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    marginBottom: 40,
  },
  disabledButton: {
    backgroundColor: colors.grayLight,
  },
  submitButtonText: {
    fontSize: 16,
    fontWeight: "600",
    color: "#FFFFFF",
  },
  infoCard: {
    backgroundColor: colors.card,
    borderRadius: 16,
    padding: 20,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  infoRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingVertical: 8,
  },
  infoLabel: {
    fontSize: 14,
    color: colors.grayMedium,
    fontWeight: "500",
  },
  infoValue: {
    fontSize: 14,
    color: colors.navy,
    fontWeight: "600",
  },
  aboutCard: {
    flexDirection: "row",
    backgroundColor: colors.card,
    borderRadius: 16,
    padding: 20,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  iconContainer: {
    width: 48,
    height: 48,
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
    marginRight: 16,
  },
  aboutContent: {
    flex: 1,
  },
  aboutTitle: {
    fontSize: 16,
    fontWeight: "700",
    color: colors.navy,
    marginBottom: 8,
  },
  aboutText: {
    fontSize: 14,
    color: colors.grayMedium,
    lineHeight: 20,
  },
  contactCard: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: colors.card,
    borderRadius: 16,
    padding: 20,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  contentContainer: {
    flex: 1,
  },
  contactLabel: {
    fontSize: 16,
    fontWeight: "600",
    color: colors.navy,
    marginBottom: 4,
  },
  contactValue: {
    fontSize: 13,
    color: colors.grayMedium,
  },
  socialContainer: {
    flexDirection: "row",
    justifyContent: "space-around",
  },
  socialButton: {
    width: 56,
    height: 56,
    borderRadius: 16,
    backgroundColor: colors.card,
    alignItems: "center",
    justifyContent: "center",
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  legalText: {
    fontSize: 12,
    color: colors.grayMedium,
    textAlign: "center",
    fontStyle: "italic",
  },
});

export default FeedbackAndAbout;

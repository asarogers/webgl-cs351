import React, { useState } from "react";
import {
  View,
  Text,
  ScrollView,
  TouchableOpacity,
  StyleSheet,
  TextInput,
  KeyboardAvoidingView,
  Platform,
  Alert,
  Linking,
} from "react-native";
import {
  Ionicons,
  MaterialCommunityIcons,
  MaterialIcons,
  Feather,
} from "@expo/vector-icons";
import SettingsHeader from "./SettingsHeader";
import { SafeAreaView } from 'react-native-safe-area-context';

const colors = {
  primary: "#3B82F6",
  primaryHover: "#2563EB",
  error: "#EF4444",
  success: "#10B981",
  navy: "#1F2937",
  slate: "#374151",
  grayMedium: "#6B7280",
  grayLight: "#E5E7EB",
  background: "#F8FAFC",
  card: "#FFFFFF",
};

const SUPPORT_INFO = {
  email: "softwareace.j@gmail.com",
  website: "https://findmyroomy.com",
  hours: "Mon–Fri, 9am–6pm EST",
  promise: "We aim to reply within 24 hours.",
};

const handleContact = (type, value) => {
  if (type === "email") Linking.openURL(`mailto:${value}`);
  if (type === "website") Linking.openURL(value);
};

export const SupportScreen = ({ setPage }) => {
  const [name, setName] = useState("");
  const [userEmail, setUserEmail] = useState("");
  const [question, setQuestion] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = () => {
    if (!name.trim() || !userEmail.trim() || !question.trim()) {
      Alert.alert("Missing Information", "Please complete all fields.");
      return;
    }
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(userEmail)) {
      Alert.alert("Invalid Email", "Please enter a valid email address.");
      return;
    }
    // Build your subject/body
    const subject = encodeURIComponent("Support Request from FindMyRoomy App");
    const body = encodeURIComponent(
      `Name: ${name}\nEmail: ${userEmail}\n\nQuestion/Issue:\n${question}\n\n---\nSent from the FindMyRoomy app`
    );
  
    // Show choice popup
    Alert.alert(
      "Send With…",
      "Which email app would you like to use?",
      [
        {
          text: "Gmail",
          onPress: () => {
            // Platform-specific Gmail compose
            let gmailUrl = "";
            if (Platform.OS === "android") {
              gmailUrl = `mailto:${SUPPORT_INFO.email}?subject=${subject}&body=${body}`;
            } else {
              // On iOS, open Gmail in browser (if app installed and user wants it, system may offer)
              gmailUrl = `googlegmail://co?to=${SUPPORT_INFO.email}&subject=${subject}&body=${body}`;
            }
            Linking.openURL(gmailUrl).catch(() => {
              // Fallback to web Gmail if app is not installed
              const webGmailUrl = `https://mail.google.com/mail/?view=cm&fs=1&to=${SUPPORT_INFO.email}&su=${subject}&body=${body}`;
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
            const mailtoUrl = `mailto:${SUPPORT_INFO.email}?subject=${subject}&body=${body}`;
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
  
  return (
    <SafeAreaView style={styles.container}>
      <SettingsHeader
        title="Support"
        subtitle="Send us a message and we'll help!"
        onBack={() => setPage("home")}
      />

      <KeyboardAvoidingView
        style={{ flex: 1 }}
        behavior={Platform.OS === "ios" ? "padding" : undefined}
      >
        <ScrollView
          showsVerticalScrollIndicator={false}
          style={styles.scrollView}
        >
          {/* Instructions */}
          <View style={styles.tipCard}>
            <MaterialCommunityIcons
              name="chat-question"
              size={24}
              color={colors.primary}
            />
            <View style={{ flex: 1, marginLeft: 16 }}>
              <Text style={styles.tipTitle}>Ask Us Anything</Text>
              <Text style={styles.tipText}>
                Fill out the form below and our support team will respond to you
                via email, usually within 24 hours.
              </Text>
            </View>
          </View>

          {/* Support Form */}
          <View style={styles.section}>
            <Text style={styles.inputLabel}>Your Name</Text>
            <TextInput
              style={styles.input}
              placeholder="Enter your name"
              value={name}
              onChangeText={setName}
              autoCapitalize="words"
              autoCorrect={false}
              placeholderTextColor={colors.grayMedium}
            />
          </View>
          <View style={styles.section}>
            <Text style={styles.inputLabel}>Your Email</Text>
            <TextInput
              style={styles.input}
              placeholder="your.email@example.com"
              value={userEmail}
              onChangeText={setUserEmail}
              keyboardType="email-address"
              autoCapitalize="none"
              autoCorrect={false}
              placeholderTextColor={colors.grayMedium}
            />
          </View>
          <View style={styles.section}>
            <Text style={styles.inputLabel}>Your Question</Text>
            <TextInput
              style={styles.inputLarge}
              placeholder="Type your question or describe your issue"
              value={question}
              onChangeText={setQuestion}
              multiline
              numberOfLines={5}
              textAlignVertical="top"
              placeholderTextColor={colors.grayMedium}
            />
          </View>
          <TouchableOpacity
            style={[
              styles.submitButton,
              (!name.trim() || !userEmail.trim() || !question.trim()) &&
                styles.disabledButton,
            ]}
            onPress={handleSubmit}
            disabled={!name.trim() || !userEmail.trim() || !question.trim()}
            activeOpacity={0.8}
          >
            <Ionicons
              name="send"
              size={18}
              color="#FFFFFF"
              style={{ marginRight: 8 }}
            />
            <Text style={styles.submitButtonText}>Submit Request</Text>
          </TouchableOpacity>

          {/* Contact Info */}
          <View style={styles.section}>
            <Text style={styles.sectionTitle}>Or contact us directly</Text>
            <TouchableOpacity
              style={styles.contactCard}
              onPress={() => handleContact("email", SUPPORT_INFO.email)}
              activeOpacity={0.7}
            >
              <View
                style={[styles.iconContainer, { backgroundColor: "#FEF3F2" }]}
              >
                <MaterialIcons name="email" size={20} color={colors.error} />
              </View>
              <View style={styles.contentContainer}>
                <Text style={styles.contactLabel}>Email</Text>
                <Text style={styles.contactValue}>{SUPPORT_INFO.email}</Text>
              </View>
              <Ionicons
                name="chevron-forward"
                size={18}
                color={colors.grayMedium}
              />
            </TouchableOpacity>
            <TouchableOpacity
              style={styles.contactCard}
              onPress={() => handleContact("website", SUPPORT_INFO.website)}
              activeOpacity={0.7}
            >
              <View
                style={[styles.iconContainer, { backgroundColor: "#F0F9FF" }]}
              >
                <Feather name="globe" size={20} color={colors.primary} />
              </View>
              <View style={styles.contentContainer}>
                <Text style={styles.contactLabel}>Website</Text>
                <Text style={styles.contactValue}>findmyroomy.com</Text>
              </View>
              <Ionicons
                name="chevron-forward"
                size={18}
                color={colors.grayMedium}
              />
            </TouchableOpacity>
          </View>

          {/* Hours & Promise */}
          <View style={styles.section}>
            <Text style={styles.sectionTitle}>Support Hours</Text>
            <View style={styles.hoursCard}>
              <MaterialCommunityIcons
                name="clock-outline"
                size={18}
                color={colors.slate}
              />
              <Text style={styles.hoursText}>
                {SUPPORT_INFO.hours}
                {"\n"}
                <Text style={{ color: colors.success, fontWeight: "700" }}>
                  {SUPPORT_INFO.promise}
                </Text>
              </Text>
            </View>
          </View>
          <View style={{ height: 48 }} />
        </ScrollView>
      </KeyboardAvoidingView>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: colors.background },
  scrollView: { flex: 1, paddingHorizontal: 20, marginTop: 20 },
  tipCard: {
    flexDirection: "row",
    alignItems: "flex-start",
    backgroundColor: colors.card,
    borderRadius: 16,
    padding: 20,
    borderWidth: 1,
    borderColor: colors.grayLight,
    shadowColor: "#000",
    shadowOpacity: 0.02,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 2 },
    elevation: 2,
    marginBottom: 18,
  },
  tipTitle: {
    fontSize: 15,
    fontWeight: "700",
    color: colors.primary,
    marginBottom: 4,
  },
  tipText: { fontSize: 13, color: colors.grayMedium, lineHeight: 19 },
  section: { marginBottom: 28 },
  sectionTitle: {
    fontSize: 13,
    fontWeight: "700",
    color: colors.slate,
    textTransform: "uppercase",
    marginBottom: 16,
    letterSpacing: 0.8,
    marginLeft: 4,
  },
  inputLabel: {
    fontSize: 14,
    fontWeight: "600",
    color: colors.navy,
    marginBottom: 8,
    marginLeft: 4,
  },
  input: {
    backgroundColor: colors.card,
    borderRadius: 12,
    padding: 16,
    fontSize: 16,
    color: colors.navy,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  inputLarge: {
    backgroundColor: colors.card,
    borderRadius: 12,
    padding: 16,
    fontSize: 16,
    color: colors.navy,
    borderWidth: 1,
    borderColor: colors.grayLight,
    minHeight: 100,
  },
  submitButton: {
    backgroundColor: colors.primary,
    borderRadius: 12,
    padding: 16,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    marginBottom: 30,
    marginTop: 12,
  },
  disabledButton: {
    backgroundColor: colors.grayLight,
  },
  submitButtonText: {
    fontSize: 16,
    fontWeight: "600",
    color: "#FFFFFF",
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
  iconContainer: {
    width: 44,
    height: 44,
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
    marginRight: 16,
  },
  contentContainer: { flex: 1 },
  contactLabel: {
    fontSize: 16,
    fontWeight: "600",
    color: colors.navy,
    marginBottom: 4,
  },
  contactValue: { fontSize: 13, color: colors.grayMedium },
  hoursCard: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: colors.card,
    borderRadius: 12,
    padding: 16,
    borderWidth: 1,
    borderColor: colors.grayLight,
  },
  hoursText: { fontSize: 13, color: colors.grayMedium, marginLeft: 8, flex: 1 },
});

export default SupportScreen;

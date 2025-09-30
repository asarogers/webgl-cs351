import React from "react";
import { ScrollView, StyleSheet, Text, View, TouchableOpacity } from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import { Ionicons } from "@expo/vector-icons";

const colors = {
  primary: "#3B82F6",
  navy: "#1F2937",
  slate: "#374151",
  grayMedium: "#6B7280",
  background: "#F8FAFC",
  card: "#FFFFFF",
  grayLight: "#E5E7EB",
};

export const TermsAndConditions = ({ onBack }) => {
  return (
    <SafeAreaView style={styles.container} edges={['top']}>
      <View style={styles.header}>
        <TouchableOpacity onPress={onBack} style={styles.closeButton}>
          <Text style={styles.closeIcon}>✕</Text>
        </TouchableOpacity>
        <Text style={styles.headerTitle}>Terms and Conditions</Text>
        <View style={{ width: 40 }} />
      </View>

      <ScrollView
        style={styles.scrollView}
        contentContainerStyle={{ paddingBottom: 40 }}
        showsVerticalScrollIndicator={false}
      >
        <Text style={styles.title}>FindMyRoomie Terms and Conditions</Text>

        <Text style={styles.paragraph}>
          Welcome to FindMyRoomie! These Terms and Conditions (the "Terms")
          govern your access to and use of our services. By creating an account,
          browsing our website, or using our mobile application, you agree to
          comply with the following Terms. Please read them carefully.
        </Text>

        <Text style={styles.heading}>1. About FindMyRoomie</Text>
        <Text style={styles.paragraph}>
          FindMyRoomie is a roommate-finding and housing search platform
          designed to help individuals connect, communicate, and arrange shared
          living opportunities. We provide tools such as user profiles, filters,
          verification tiers, messaging, and map-based search.
        </Text>

        <Text style={styles.heading}>2. Services Offered</Text>
        <Text style={styles.subheading}>2.1 Roommate Matching</Text>
        <Text style={styles.paragraph}>
          Users can browse profiles, apply filters, and use compatibility
          indicators to identify potential roommates.
        </Text>
        <Text style={styles.subheading}>2.2 Communication Tools</Text>
        <Text style={styles.paragraph}>
          Our platform provides in-app messaging and profile favoriting. We may
          also offer optional premium features such as advanced filters,
          verification badges, or priority placement.
        </Text>
        <Text style={styles.subheading}>2.3 Subscription Services</Text>
        <Text style={styles.paragraph}>
          Certain features may be available only to paid subscribers.
          Subscriptions are billed monthly or annually, depending on the plan
          selected.
        </Text>

        <Text style={styles.heading}>3. Account Registration</Text>
        <Text style={styles.subheading}>3.1 Eligibility</Text>
        <Text style={styles.paragraph}>
          You must be at least 18 years old to create a FindMyRoomie account. By
          registering, you confirm that the information you provide is accurate
          and that you have the legal capacity to enter into this Agreement.
        </Text>
        <Text style={styles.subheading}>3.2 Account Security</Text>
        <Text style={styles.paragraph}>
          You are responsible for maintaining the confidentiality of your login
          credentials. FindMyRoomie is not liable for unauthorized access caused
          by your failure to secure your account.
        </Text>

        <Text style={styles.heading}>4. User Conduct</Text>
        <Text style={styles.paragraph}>
          You agree to use FindMyRoomie responsibly and in compliance with all
          applicable laws. Prohibited activities include harassment, fraud,
          impersonation, and posting false or misleading information.
        </Text>

        <Text style={styles.heading}>5. Content and Intellectual Property</Text>
        <Text style={styles.paragraph}>
          You retain ownership of content you post, but grant FindMyRoomie a
          license to use, display, and distribute that content on our platform.
        </Text>

        <Text style={styles.heading}>6. Termination</Text>
        <Text style={styles.paragraph}>
          We reserve the right to suspend or terminate accounts that violate
          these Terms or engage in harmful behavior.
        </Text>

        <Text style={styles.heading}>7. Contact Information</Text>
        <Text style={styles.paragraph}>
          If you have questions about these Terms, please contact us at:
          {"\n"}FindMyRoomie{"\n"}Email: aethurtech@gmail.com
        </Text>
      </ScrollView>
    </SafeAreaView>
  );
};
const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: colors.background,
    paddingTop: 50,
  },
  header: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    paddingHorizontal: 16,
    paddingVertical: 16,
    borderBottomWidth: 1,
    borderBottomColor: colors.grayLight,
    backgroundColor: colors.card,
  },
  closeButton: {
    width: 40,
    height: 40,
    alignItems: "center",
    justifyContent: "center",
    borderRadius: 20,
    backgroundColor: colors.grayLight,
  },
  closeIcon: {
    fontSize: 20,
    fontWeight: "600",
    color: colors.navy,
  },
  headerTitle: {
    fontSize: 18,
    fontWeight: "600",
    color: colors.navy,
  },
  scrollView: {
    flex: 1,
    paddingHorizontal: 20,
    marginTop: 10,
  },
  title: {
    fontSize: 20,
    fontWeight: "700",
    color: colors.navy,
    marginBottom: 20,
  },
  heading: {
    fontSize: 16,
    fontWeight: "700",
    color: colors.slate,
    marginTop: 20,
    marginBottom: 8,
  },
  subheading: {
    fontSize: 14,
    fontWeight: "600",
    color: colors.navy,
    marginTop: 12,
    marginBottom: 4,
  },
  paragraph: {
    fontSize: 14,
    color: colors.grayMedium,
    lineHeight: 20,
    marginBottom: 10,
  },
});


export default TermsAndConditions;

import React from "react";
import {
  ScrollView,
  StyleSheet,
  Text,
  View,
  TouchableOpacity,
} from "react-native";
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

export const PrivacyPolicy = ({ onBack }) => {
    return (
      <SafeAreaView style={styles.container}>
        <View style={styles.header}>
          <TouchableOpacity onPress={onBack} style={styles.closeButton}>
            <Text style={styles.closeIcon}>✕</Text>
          </TouchableOpacity>
          <Text style={styles.headerTitle}>Privacy Policy</Text>
          <View style={{ width: 40 }} />
        </View>
  
        <ScrollView
          style={styles.scrollView}
          contentContainerStyle={{ paddingBottom: 40 }}
          showsVerticalScrollIndicator={false}
        >
          <Text style={styles.title}>FindMyRoomie Privacy Policy</Text>
  
          <Text style={styles.paragraph}>
            At FindMyRoomie, your privacy is important to us. This Privacy Policy
            explains how we collect, use, store, and protect your personal
            information when you use our services.
          </Text>
  
          <Text style={styles.heading}>1. Information We Collect</Text>
          <Text style={styles.subheading}>1.1 Account Information</Text>
          <Text style={styles.paragraph}>
            When you create an account, we collect information such as your name,
            email address, phone number, date of birth, and profile photo.
          </Text>
  
          <Text style={styles.subheading}>1.2 Profile Data</Text>
          <Text style={styles.paragraph}>
            You may provide additional information such as location preferences,
            lifestyle habits, budget range, and other details to help match you
            with compatible roommates.
          </Text>
  
          <Text style={styles.subheading}>1.3 Usage Data</Text>
          <Text style={styles.paragraph}>
            We collect information about how you interact with our platform,
            including pages viewed, features used, and time spent on the app.
          </Text>
  
          <Text style={styles.heading}>2. How We Use Your Information</Text>
          <Text style={styles.paragraph}>
            We use your information to: provide and improve our services, match
            you with potential roommates, communicate with you about your account,
            ensure platform safety and security, and comply with legal obligations.
          </Text>
  
          <Text style={styles.heading}>3. Information Sharing</Text>
          <Text style={styles.paragraph}>
            We do not sell your personal information. We may share information with
            service providers, for legal compliance, or with your explicit consent.
          </Text>
  
          <Text style={styles.heading}>4. Data Security</Text>
          <Text style={styles.paragraph}>
            We implement industry-standard security measures to protect your data,
            including encryption, secure servers, and regular security audits.
          </Text>
  
          <Text style={styles.heading}>5. Your Rights</Text>
          <Text style={styles.paragraph}>
            You have the right to access, correct, or delete your personal
            information. You can also control your privacy settings and profile
            visibility within the app.
          </Text>
  
          <Text style={styles.heading}>6. Contact Us</Text>
          <Text style={styles.paragraph}>
            If you have questions about this Privacy Policy, please contact us at:
            {"\n"}Email: aethurtech@gmail.com
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
  

export default PrivacyPolicy;
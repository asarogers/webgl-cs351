import { Ionicons, MaterialIcons } from "@expo/vector-icons";
import React, { useEffect, useState } from "react";
import {
  ActivityIndicator,
  Alert,

  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { getStepByNumber, navigateTo } from "../../../../components/onboardingStepsHelper";
import authService from "../../../../database/authService";

const PhoneAndEmailVerificationScreen = ({ navigation }) => {
  const [email, setEmail] = useState("");
  const [emailVerified, setEmailVerified] = useState(false);
  const [checking, setChecking] = useState(false);
  const [resending, setResending] = useState(false);

  // On mount, load the email and verification status
  useEffect(() => {
    const fetchUserAndVerification = async () => {
      setChecking(true);
      const currentUser = await authService.getCurrentUser();
      if (currentUser?.success && currentUser.user?.email) {
        setEmail(currentUser.user.email);
      }
      const verification = await authService.checkEmailVerification();
      setEmailVerified(verification.isVerified);
      setChecking(false);
    };
    fetchUserAndVerification();
  }, []);

  const handleRefresh = async () => {
    setChecking(true);
    const verification = await authService.checkEmailVerification();
    setEmailVerified(verification.isVerified);
    setChecking(false);
    if (verification.isVerified) {
      Alert.alert("Success", "Your email is verified!");
    }
  };

  const handleResend = async () => {
    setResending(true);
    await authService.resendEmailVerification(email);
    setResending(false);
    Alert.alert("Verification Sent", "A verification email has been sent to your inbox.");
  };

  const handleContinue = async () => {
    try {
      setChecking(true); 
      await authService.setUserVerificationBronze();
      const currentUser = await authService.getCurrentUser();
      const userId = currentUser?.user?.id;
      if (!userId) {
        Alert.alert("Error", "User not found.");
        setChecking(false);
        return;
      }
    
      const onboarding = getStepByNumber(1); 
      await authService.setOnboardingStep(onboarding);
      setChecking(false);
      navigateTo(onboarding, navigation);
    } catch (e) {
      setChecking(false);
      Alert.alert("Error", "Could not continue onboarding. Please try again.");
      console.error(e);
    }
  };
  
  return (
    <SafeAreaView style={styles.safeArea}>
      <ScrollView contentContainerStyle={styles.container}>
        <View style={styles.headerRow}>
          <TouchableOpacity
            style={styles.backButton}
            onPress={() => navigation.goBack()}
            hitSlop={{ top: 10, left: 10, bottom: 10, right: 10 }}
          >
            <Ionicons name="arrow-back" size={24} color="#60A5FA" />
          </TouchableOpacity>
          <Text style={styles.title}>Verify your email</Text>
          <TouchableOpacity onPress={() => navigation.goBack()}>
            <Text style={styles.skip}>Skip</Text>
          </TouchableOpacity>
        </View>

        <Text style={styles.subtitle}>
          We’ve sent a verification email to:
        </Text>
        <Text style={styles.email}>{email}</Text>

        <View style={styles.section}>
          <View style={styles.row}>
            <MaterialIcons name="email" size={18} color="#9CA3AF" />
            <Text style={styles.label}>Email Status</Text>
          </View>
          <Text style={[
            styles.status,
            { color: emailVerified ? "#22c55e" : "#fbbf24" }
          ]}>
            {emailVerified ? "✅ Email verified!" : "⏳ Waiting for email verification..."}
          </Text>

          <TouchableOpacity
            style={[
              styles.verifyButton,
              emailVerified && { backgroundColor: "#22c55e" },
              checking && { opacity: 0.7 }
            ]}
            onPress={handleRefresh}
            disabled={checking || emailVerified}
          >
            {checking ? (
              <ActivityIndicator color="#fff" />
            ) : (
              <Text style={styles.verifyText}>
                {emailVerified ? "Verified" : "Refresh status"}
              </Text>
            )}
          </TouchableOpacity>

          <TouchableOpacity
            style={{ marginTop: 10, opacity: resending ? 0.6 : 1 }}
            onPress={handleResend}
            disabled={resending}
          >
            <Text style={styles.resendText}>Resend verification email</Text>
          </TouchableOpacity>
        </View>

        <Text style={styles.info}>
          {emailVerified
            ? "🎉 You’re all set! Tap continue."
            : "🟡 Please verify your email before continuing."}
        </Text>

        <TouchableOpacity
          style={[
            styles.ctaButton,
            !emailVerified && { opacity: 0.6 }
          ]}
          onPress={() => {
            if (emailVerified) {
              handleContinue()
            } else {
              Alert.alert("Not Verified", "Please verify your email to continue.");
            }
          }}
        >
          <Text style={styles.ctaText}>
            Continue
          </Text>
        </TouchableOpacity>
      </ScrollView>
    </SafeAreaView>
  );
};

export default PhoneAndEmailVerificationScreen;

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: "#1F2937",
  },
  container: {
    padding: 24,
    flexGrow: 1,
    justifyContent: "center",
  },
  headerRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 18,
  },
  backButton: {
    marginRight: 12,
    padding: 2,
    alignItems: "center",
    justifyContent: "center",
  },
  skip: {
    color: "#60A5FA",
    fontSize: 14,
  },
  title: {
    fontSize: 24,
    fontWeight: "700",
    color: "#FFFFFF",
  },
  subtitle: {
    color: "#9CA3AF",
    marginTop: 2,
    marginBottom: 2,
    fontSize: 14,
    textAlign: "center",
  },
  email: {
    color: "#60A5FA",
    fontSize: 16,
    fontWeight: "600",
    textAlign: "center",
    marginBottom: 18,
  },
  section: {
    backgroundColor: "#111827",
    borderRadius: 10,
    padding: 20,
    marginBottom: 18,
  },
  row: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
    marginBottom: 6,
  },
  label: {
    color: "#9CA3AF",
    fontSize: 14,
    marginLeft: 6,
  },
  status: {
    fontWeight: "600",
    fontSize: 16,
    marginBottom: 16,
    marginTop: 6,
  },
  verifyButton: {
    backgroundColor: "#3B82F6",
    paddingVertical: 12,
    borderRadius: 8,
    alignItems: "center",
    marginBottom: 4,
  },
  verifyText: {
    color: "#FFFFFF",
    fontWeight: "600",
    fontSize: 16,
  },
  resendText: {
    color: "#60A5FA",
    fontSize: 14,
    textAlign: "center",
    marginTop: 0,
  },
  info: {
    color: "#FBBF24",
    backgroundColor: "#1F2937",
    fontSize: 14,
    marginBottom: 18,
    paddingHorizontal: 6,
    textAlign: "center",
  },
  ctaButton: {
    backgroundColor: "#3B82F6",
    paddingVertical: 16,
    borderRadius: 8,
    alignItems: "center",
    marginBottom: 10,
  },
  ctaText: {
    color: "#fff",
    fontWeight: "600",
    fontSize: 17,
  },
});

import React, { useState, useCallback } from "react";
import {
  View,
  Text,
  StyleSheet,
  TouchableOpacity,
  ScrollView,
  Dimensions,
  KeyboardAvoidingView,
  Platform,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { Ionicons } from "@expo/vector-icons";

const { width } = Dimensions.get("window");

export default function ProgressBar({
  currentStep,
  totalSteps,
  onSkip,
  onNext,
  goBack,
  canProceed,
  children,
  navigation,
}) {
  const [isProcessing, setIsProcessing] = useState(false);
  const progress = Math.max(0, Math.min(1, (currentStep + 1) / totalSteps));

  // Button is always enabled on last step
  const allowProceed = canProceed || currentStep === totalSteps;

  const handleContinue = useCallback(() => {
    if (allowProceed && onNext && !isProcessing) {
      setIsProcessing(true);
      onNext();
      setTimeout(() => setIsProcessing(false), 300);
    }
  }, [allowProceed, onNext, isProcessing]);

  const handleSkip = () => {
    if (onSkip) onSkip();
    else if (onNext) onNext();
  };

  const handleBack = () => {
    if (goBack) goBack();
  };

  // Always show button (for step 0 up to totalSteps - 1)
  const showContinueButton = currentStep <= totalSteps;

  // Show "Finish" on last step, "Continue" otherwise
  const buttonLabel = currentStep === totalSteps ? "Finish" : "Continue";

  return (
    <SafeAreaView style={styles.safeArea}>
      {/* Header */}
      <View style={styles.header}>
        <TouchableOpacity onPress={handleBack} style={styles.backButton}>
          <Ionicons name="chevron-back" size={24} color="#374151" />
        </TouchableOpacity>
        <TouchableOpacity onPress={handleSkip} style={styles.skipButton}>
          <Text style={styles.skipText}>Skip</Text>
        </TouchableOpacity>
      </View>

      {/* Progress at the top */}
      <View style={styles.progressContainer}>
        <View style={styles.progressBar}>
          <View
            style={[
              styles.progressFill,
              { width: `${Math.round(progress * 100)}%` },
            ]}
          />
        </View>
        <Text style={styles.stepText}>
          Step {currentStep + 1} of {totalSteps+1}
        </Text>
      </View>

      {/* Main Content */}
      <ScrollView
        style={styles.content}
        contentContainerStyle={{ paddingBottom: 140 }}
        showsVerticalScrollIndicator={false}
      >
        {children}
      </ScrollView>

      {/* Sticky bottom: Continue/Finish button */}
      <KeyboardAvoidingView
        behavior={Platform.OS === "ios" ? "padding" : undefined}
        style={styles.stickyArea}
      >
        {showContinueButton && (
          <View style={styles.buttonContainer}>
            <TouchableOpacity
              style={[
                styles.continueButton,
                !allowProceed && styles.continueDisabled,
              ]}
              onPress={handleContinue}
              disabled={!allowProceed}
              activeOpacity={0.8}
            >
              <Text
                style={[
                  styles.continueText,
                  !allowProceed && styles.continueTextDisabled,
                ]}
              >
                {buttonLabel}
              </Text>
              <Ionicons
                name="arrow-forward"
                size={16}
                color={allowProceed ? "white" : "#9CA3AF"}
                style={styles.continueIcon}
              />
            </TouchableOpacity>
          </View>
        )}
      </KeyboardAvoidingView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: "#F8FAFC",
    width: width,
  },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingHorizontal: 24,
    paddingTop: 4,
  },
  backButton: {
    padding: 8,
  },
  skipButton: {
    padding: 8,
  },
  skipText: {
    fontSize: 16,
    color: "#6B7280",
    fontWeight: "600",
  },
  content: {
    flex: 1,
    paddingHorizontal: 24,
    paddingTop: 12,
  },
  stickyArea: {
    position: "absolute",
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: "#F8FAFC",
    borderTopLeftRadius: 16,
    borderTopRightRadius: 16,
    paddingHorizontal: 24,
    paddingTop: 18,
    paddingBottom: 18,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: -2 },
    shadowOpacity: 0.07,
    shadowRadius: 10,
    elevation: 12,
  },
  buttonContainer: {
    width: "100%",
    marginBottom: 20,
  },
  continueButton: {
    width: "100%",
    backgroundColor: "#3B82F6",
    paddingVertical: 16,
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
    flexDirection: "row",
    marginBottom: 10,
    shadowColor: "#3B82F6",
    shadowOffset: {
      width: 0,
      height: 4,
    },
    shadowOpacity: 0.2,
    shadowRadius: 8,
    elevation: 4,
  },
  continueDisabled: {
    backgroundColor: "#E5E7EB",
    shadowOpacity: 0,
  },
  continueText: {
    color: "white",
    fontWeight: "600",
    fontSize: 16,
  },
  continueTextDisabled: {
    color: "#9CA3AF",
  },
  continueIcon: {
    marginLeft: 8,
  },
  progressContainer: {
    alignItems: "center",
    marginTop: 2,
  },
  progressBar: {
    width: "100%",
    height: 4,
    backgroundColor: "#E5E7EB",
    borderRadius: 2,
    marginBottom: 4,
  },
  progressFill: {
    height: "100%",
    backgroundColor: "#3B82F6",
    borderRadius: 2,
  },
  stepText: {
    fontSize: 12,
    color: "#9CA3AF",
    fontWeight: "500",
  },
});

import { Ionicons } from "@expo/vector-icons";
import React, { useState } from "react";
import {
  ActivityIndicator,
  Animated,
  StyleSheet,
  Text,
  TouchableOpacity,
  View
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { getStepByNumber, navigateTo } from "../../../../components/onboardingStepsHelper";
import authService from '../../../../database/authService'; // Adjust path if needed

const intents = [
  {
    id: "share",
    title: "Looking for a roommate to share a place with",
    description: "Find someone to split rent and live together",
    icon: "people-outline",
    color: "#3B82F6",
  },
  {
    id: "host",
    title: "I have a place, looking for someone to move in",
    description: "You already have housing and need a roommate",
    icon: "home-outline",
    color: "#10B981",
  },
  {
    id: "browse",
    title: "Just browsing for now",
    description: "Exploring options and seeing what's available",
    icon: "search-outline",
    color: "#64748B",
  },
];

export default function UserIntentScreen({ navigation }) {
  const [selectedIntent, setSelectedIntent] = useState(null);
  const [animatedValue] = useState(new Animated.Value(0));
  const [isSaving, setIsSaving] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  const totalSteps = 7;
  const currentStep = 1;
  const progress = Math.max(0, Math.min(1, currentStep / totalSteps));

  // Animate on mount
  React.useEffect(() => {
    Animated.timing(animatedValue, {
      toValue: 1,
      duration: 800,
      useNativeDriver: true,
    }).start();
  }, []);

  // Fetch user's saved intent on mount
  React.useEffect(() => {
    let mounted = true;
    const fetchIntent = async () => {
      setIsLoading(true);
      try {
        if (authService.getUserIntent) {
          const intent = await authService.getUserIntent();
          if (intent && mounted) setSelectedIntent(intent);
        }
      } catch (e) {
        // You can display an error here if you want
      }
      setIsLoading(false);
    };
    fetchIntent();
    return () => { mounted = false; };
  }, []);

  const handleContinue = async () => {
    if (selectedIntent) {
      setIsSaving(true);
      const result = await authService.saveUserIntent(selectedIntent);
      setIsSaving(false);
      if (!result.success) {
        alert("Error saving your intent. Please try again.");
        return;
      }
      const onboarding = getStepByNumber(2)
      await authService.setOnboardingStep(onboarding);
      navigateTo(onboarding, navigation)
    }
  };

  const handleSkip = () => {
    navigation.navigate("Quiz", { intent: null });
  };

  const handleBack = () => {
    navigation.goBack();
  };

  if (isLoading) {
    return (
      <View style={{ flex: 1, alignItems: "center", justifyContent: "center" }}>
        <ActivityIndicator size="large" color="#3B82F6" />
      </View>
    );
  }

  return (
    <SafeAreaView style={styles.container}>
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
          Step {currentStep} of {totalSteps}
        </Text>
      </View>

      <Animated.View
        style={[
          styles.content,
          {
            opacity: animatedValue,
            transform: [
              {
                translateY: animatedValue.interpolate({
                  inputRange: [0, 1],
                  outputRange: [20, 0],
                }),
              },
            ],
          },
        ]}
      >
        <View style={styles.iconContainer}>
          <View style={styles.iconBackground}>
            <Ionicons name="person-add-outline" size={32} color="#3B82F6" />
          </View>
        </View>

        <Text style={styles.heading}>What brings you here?</Text>
        <Text style={styles.subheading}>
          This helps us personalize your experience and show you the most
          relevant matches
        </Text>

        <View style={styles.options}>
          {intents.map((item, index) => (
            <Animated.View
              key={item.id}
              style={{
                opacity: animatedValue,
                transform: [
                  {
                    translateY: animatedValue.interpolate({
                      inputRange: [0, 1],
                      outputRange: [30 + index * 10, 0],
                    }),
                  },
                ],
              }}
            >
              <TouchableOpacity
                style={[
                  styles.optionCard,
                  selectedIntent === item.id && styles.optionCardSelected,
                ]}
                onPress={() => setSelectedIntent(item.id)}
                activeOpacity={0.8}
              >
                <View style={styles.optionHeader}>
                  <View
                    style={[
                      styles.optionIcon,
                      { backgroundColor: `${item.color}15` },
                    ]}
                  >
                    <Ionicons name={item.icon} size={20} color={item.color} />
                  </View>
                  {selectedIntent === item.id && (
                    <View style={styles.checkmark}>
                      <Ionicons
                        name="checkmark-circle"
                        size={20}
                        color="#3B82F6"
                      />
                    </View>
                  )}
                </View>
                <Text style={styles.optionTitle}>{item.title}</Text>
                <Text style={styles.optionDescription}>{item.description}</Text>
              </TouchableOpacity>
            </Animated.View>
          ))}
        </View>

        <View style={styles.buttonContainer}>
          <TouchableOpacity
            style={[
              styles.continueButton,
              !selectedIntent && styles.continueDisabled,
            ]}
            onPress={handleContinue}
            disabled={!selectedIntent || isSaving}
            activeOpacity={0.8}
          >
            <Text
              style={[
                styles.continueText,
                !selectedIntent && styles.continueTextDisabled,
              ]}
            >
              {isSaving ? <ActivityIndicator color="white" /> : "Continue"}
            </Text>
            <Ionicons
              name="arrow-forward"
              size={16}
              color={selectedIntent ? "white" : "#9CA3AF"}
              style={styles.continueIcon}
            />
          </TouchableOpacity>
        </View>
      </Animated.View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#F8FAFC",
  },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    paddingHorizontal: 24,
  },
  backButton: {
    padding: 8,
  },
  skipButton: {
    padding: 8,
  },
  skipText: {
    color: "#6B7280",
    fontSize: 16,
    fontWeight: "500",
  },
  content: {
    flex: 1,
    paddingHorizontal: 24,
    paddingTop: 12,
  },
  iconContainer: {
    alignItems: "center",
    marginBottom: 2,
  },
  iconBackground: {
    width: 64,
    height: 64,
    borderRadius: 32,
    backgroundColor: "#EEF2FF",
    justifyContent: "center",
    alignItems: "center",
  },
  heading: {
    fontSize: 24,
    fontWeight: "700",
    textAlign: "center",
    color: "#111827",
    marginBottom: 12,
  },
  subheading: {
    textAlign: "center",
    color: "#6B7280",
    fontSize: 16,
    lineHeight: 24,
    marginBottom: 32,
    paddingHorizontal: 8,
  },
  options: {
    gap: 16,
    marginBottom: 32,
  },
  optionCard: {
    backgroundColor: "white",
    padding: 20,
    borderRadius: 12,
    borderWidth: 2,
    borderColor: "#E5E7EB",
    shadowColor: "#000",
    shadowOffset: {
      width: 0,
      height: 1,
    },
    shadowOpacity: 0.05,
    shadowRadius: 3,
    elevation: 2,
  },
  optionCardSelected: {
    borderColor: "#3B82F6",
    backgroundColor: "#F0F9FF",
  },
  optionHeader: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 0,
  },
  optionIcon: {
    width: 36,
    height: 36,
    borderRadius: 18,
    justifyContent: "center",
    alignItems: "center",
  },
  checkmark: {
    marginRight: 4,
  },
  optionTitle: {
    fontWeight: "600",
    color: "#111827",
    fontSize: 16,
    marginBottom: 6,
  },
  optionDescription: {
    fontSize: 14,
    color: "#6B7280",
    lineHeight: 20,
  },
  buttonContainer: {
    marginTop: "auto",
    paddingBottom: 16,
  },
  continueButton: {
    backgroundColor: "#3B82F6",
    paddingVertical: 16,
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
    flexDirection: "row",
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
    paddingHorizontal: 24,
    alignItems: "center",
  },
  progressBar: {
    width: "100%",
    height: 4,
    backgroundColor: "#E5E7EB",
    borderRadius: 2,
    marginBottom: 8,
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

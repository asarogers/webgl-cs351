import React from "react";
import {
  View,
  Text,
  StyleSheet,
  TouchableOpacity,
  Dimensions,
  ScrollView,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { FontAwesome5, Feather } from "@expo/vector-icons";
import { useNavigation } from "@react-navigation/native";
import AsyncStorage from "@react-native-async-storage/async-storage";

const { width } = Dimensions.get("window");

const levels = [
  {
    title: "Bronze",
    subtitle: "Basic verification",
    bg: "#ECFDF5",
    color: "#10B981",
    icon: "mobile-alt",
    items: ["Phone + Email Verification"],
  },
  {
    title: "Silver",
    subtitle: "Identity verified",
    bg: "#EFF6FF",
    color: "#3B82F6",
    icon: "id-badge",
    items: ["Bronze + Government ID + Selfie"],
  },
  {
    title: "Gold",
    subtitle: "Financial verification",
    bg: "#FFFBEB",
    color: "#F59E0B",
    icon: "wallet",
    items: ["Silver + Income Verification"],
  },
  {
    title: "Platinum",
    subtitle: "Full verification",
    bg: "#EEF2FF",
    color: "#6366F1",
    icon: "medal",
    items: ["Gold + Background Check"],
  },
];

const VerificationLevels = ({ onSkip, handleNext, onVerify, completeOnboarding }) => {
  const navigation = useNavigation();

  return (
    <SafeAreaView style={styles.safeArea}>
      <View style={styles.container}>
        <ScrollView
          contentContainerStyle={styles.scrollContainer}
          showsVerticalScrollIndicator={false}
        >
          <Text style={styles.title}>Verification Levels</Text>
          <Text style={styles.subtitle}>
            Build trust and increase your match rate with verified credentials
          </Text>

          {levels.map((level, idx) => (
            <View
              key={idx}
              style={[styles.card, { backgroundColor: level.bg }]}
            >
              <View style={styles.cardHeader}>
                <FontAwesome5
                  name={level.icon}
                  size={16}
                  color={level.color}
                  style={styles.cardIcon}
                />
                <View style={styles.cardTitleContainer}>
                  <Text style={[styles.cardTitle, { color: level.color }]}>
                    {level.title}
                  </Text>
                  <Text style={styles.cardSubtitle}>{level.subtitle}</Text>
                </View>
              </View>
              {level.items.map((item, i) => (
                <View key={i} style={styles.bulletItem}>
                  <Feather name="check-circle" size={14} color="#10B981" />
                  <Text style={styles.bulletText}>{item}</Text>
                </View>
              ))}
            </View>
          ))}

          <View style={styles.footerBox}>
            <Text style={styles.footerTitle}>Benefits</Text>
            <Text style={styles.footerDescription}>
              Higher verification levels increase your match rate by 3× and help
              you find trusted roommates faster.
            </Text>
          </View>
        </ScrollView>

        {/* Carousel-style Footer */}
        <View style={styles.carouselFooter}>
          <TouchableOpacity
            style={styles.skipButton}
            onPress={onSkip}
            activeOpacity={0.7}
          >
            <Text style={styles.skipText}>Skip</Text>
          </TouchableOpacity>

          <View style={styles.dotsContainer}>
            {[0, 1, 2, 3].map((i) => (
              <View
                key={i}
                style={[
                  styles.dot,
                  i === 3 && styles.activeDot, // highlight the last dot
                ]}
              />
            ))}
          </View>

          <TouchableOpacity
            style={styles.verifyButton}
            onPress={ () => {
              completeOnboarding(); // or from route.params if you're using navigation params
              navigation.replace("Auth", { screen: "Login" });
            }}
            activeOpacity={0.8}
          >
            <Text style={styles.verifyText}>Start Verification</Text>
          </TouchableOpacity>
        </View>
      </View>
    </SafeAreaView>
  );
};

export default VerificationLevels;

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: "#1F2937",
  },
  container: {
    flex: 1,
    width,
    position: "relative",
  },
  scrollContainer: {
    padding: 16,
    paddingBottom: 120,
  },
  title: {
    fontSize: 20,
    fontWeight: "700",
    color: "#FFFFFF",
    textAlign: "center",
    marginBottom: 6,
  },
  subtitle: {
    fontSize: 13,
    color: "#9CA3AF",
    textAlign: "center",
    marginBottom: 20,
    lineHeight: 18,
  },
  card: {
    borderRadius: 10,
    padding: 12,
    marginBottom: 10,
  },
  cardHeader: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 8,
  },
  cardIcon: {
    marginRight: 10,
  },
  cardTitleContainer: {
    flex: 1,
  },
  cardTitle: {
    fontSize: 15,
    fontWeight: "700",
  },
  cardSubtitle: {
    fontSize: 12,
    color: "#6B7280",
    marginTop: 1,
  },
  bulletItem: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
    marginBottom: 4,
    paddingLeft: 2,
  },
  bulletText: {
    fontSize: 13,
    color: "#374151",
    flex: 1,
  },
  footerBox: {
    backgroundColor: "#111827",
    padding: 12,
    borderRadius: 10,
    marginTop: 6,
    marginBottom: 16,
  },
  footerTitle: {
    color: "#60A5FA",
    fontSize: 13,
    fontWeight: "600",
    marginBottom: 4,
  },
  footerDescription: {
    color: "#D1D5DB",
    fontSize: 12,
    lineHeight: 16,
  },

  // 🧠 Carousel Footer Style
  carouselFooter: {
    position: "absolute",
    bottom: 0,
    width: "100%",
    flexDirection: "row",
    paddingHorizontal: 24,
    paddingVertical: 16,
    backgroundColor: "#1F2937",
    justifyContent: "space-between",
    alignItems: "center",
  },
  skipButton: {
    minWidth: 60,
    paddingVertical: 8,
    paddingHorizontal: 12,
    borderRadius: 20,
    backgroundColor: "rgba(255, 255, 255, 0.1)",
    alignItems: "center",
    justifyContent: "center",
  },
  skipText: {
    color: "#BFDBFE",
    fontSize: 14,
    fontWeight: "500",
  },
  dotsContainer: {
    flexDirection: "row",
    gap: 8,
    alignItems: "center",
    justifyContent: "center",
    flex: 1,
  },
  dot: {
    width: 8,
    height: 8,
    borderRadius: 4,
    backgroundColor: "#60A5FA",
    opacity: 0.5,
  },
  activeDot: {
    width: 24,
    backgroundColor: "#FFFFFF",
    opacity: 1,
  },

  verifyButton: {
    minWidth: 60,
    paddingVertical: 8,
    paddingHorizontal: 16,
    borderRadius: 20,
    backgroundColor: "#FFFFFF",
    alignItems: "center",
    justifyContent: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 3,
  },
  verifyText: {
    color: "#1E40AF",
    fontSize: 14,
    fontWeight: "600",
  },
});

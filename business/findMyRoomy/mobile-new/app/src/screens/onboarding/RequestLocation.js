import { Feather, Ionicons, MaterialIcons } from "@expo/vector-icons";
import { LinearGradient } from "expo-linear-gradient";
import * as Location from "expo-location";
import React, { useEffect, useRef } from "react";
import {
  Alert,
  Animated,
  Dimensions,
  StatusBar,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { getStepByNumber } from "../../../../components/onboardingStepsHelper";
import authService from "../../../../database/authService";

const { width, height } = Dimensions.get("window");

export default function RequestLocation({ navigation }) {
  const fadeAnim = useRef(new Animated.Value(0)).current;
  const slideAnim = useRef(new Animated.Value(30)).current;
  const scaleAnim = useRef(new Animated.Value(0.95)).current;

  useEffect(() => {
    Animated.parallel([
      Animated.timing(fadeAnim, { toValue: 1, duration: 600, useNativeDriver: true }),
      Animated.timing(slideAnim, { toValue: 0, duration: 500, useNativeDriver: true }),
      Animated.spring(scaleAnim, { toValue: 1, tension: 100, friction: 8, useNativeDriver: true }),
    ]).start();
  }, []);

  const handleEnableLocation = async () => {
    try {
      let { status } = await Location.requestForegroundPermissionsAsync();
      if (status === "granted") {
        let location = await Location.getCurrentPositionAsync({});
        // Save location sharing as true
        await authService.setLocationSharing(true);
        navigation.replace("Main", { screen: "Map", params: { userLocation: location.coords } });
      } else {
        // Save as false (not sharing)
        await authService.setLocationSharing(false);
        navigation.replace("Main", { screen: "Map" });
      }
  
      const onboarding = getStepByNumber(9)
      await authService.setOnboardingStep(onboarding);
    } catch (e) {
      Alert.alert("Error", "An error occurred while accessing location.");
      await authService.setLocationSharing(false); // Defensive fallback
    }
  };
  
  const handleSkip = async () => {
    // User chose not to share location
    await authService.setLocationSharing(false);
    navigation.replace("Main", { screen: "Map" });
  };

  const InfoRow = ({
    icon,
    iconColor,
    title,
    titleColor,
    description,
    delay = 0,
  }) => {
    const itemFadeAnim = useRef(new Animated.Value(0)).current;
    const itemSlideAnim = useRef(new Animated.Value(20)).current;
    useEffect(() => {
      const timer = setTimeout(() => {
        Animated.parallel([
          Animated.timing(itemFadeAnim, { toValue: 1, duration: 400, useNativeDriver: true }),
          Animated.timing(itemSlideAnim, { toValue: 0, duration: 400, useNativeDriver: true }),
        ]).start();
      }, delay);
      return () => clearTimeout(timer);
    }, [delay]);

    return (
      <Animated.View
        style={[
          styles.infoRow,
          {
            opacity: itemFadeAnim,
            transform: [{ translateY: itemSlideAnim }],
          },
        ]}
      >
        <View style={[styles.infoIconContainer, { backgroundColor: `${iconColor}15`, height: 24 }]}>
          {icon}
        </View>
        <View style={styles.infoTextContainer}>
          <Text style={[styles.infoTitle, { color: titleColor }]}>{title}</Text>
          <Text style={styles.infoDesc}>{description}</Text>
        </View>
      </Animated.View>
    );
  };

  return (
    <SafeAreaView style={styles.safeArea}>
      <StatusBar barStyle="dark-content" backgroundColor="#F8FAFC" />
      <LinearGradient
        colors={['rgba(59,130,246,0.03)', '#F8FAFC']}
        style={styles.backgroundGradient}
        start={{ x: 0.5, y: 0 }}
        end={{ x: 0.5, y: 1 }}
      />
      <Animated.View
        style={[
          styles.container,
          {
            opacity: fadeAnim,
            transform: [{ translateY: slideAnim }, { scale: scaleAnim }],
          },
        ]}
      >
        <View style={styles.card}>
          <View style={styles.iconSection}>
            <Animated.View style={[styles.iconCircle, styles.iconPulse]}>
              <View style={styles.iconInner}>
                <Ionicons name="location" size={32} color="#3B82F6" />
              </View>
            </Animated.View>
          </View>
          <View style={styles.textSection}>
            <Text style={styles.header}>Find your perfect roommate match</Text>
            <Text style={styles.subHeader}>
              Discover roommates in your area and explore neighborhoods that fit your lifestyle and budget.
            </Text>
          </View>
          <View style={styles.infoList}>
            <InfoRow
              icon={<Ionicons name="people" size={18} color="#3B82F6" />}
              iconColor="#3B82F6"
              title="Smart nearby matches"
              titleColor="#1F2937"
              description="Connect with compatible roommates in your preferred areas"
              delay={200}
            />
            <InfoRow
              icon={<Feather name="map-pin" size={18} color="#10B981" />}
              iconColor="#10B981"
              title="Custom search zones"
              titleColor="#1F2937"
              description="Draw and save areas that match your commute and budget"
              delay={300}
            />
            <InfoRow
              icon={<MaterialIcons name="verified_user" size={18} color="#8B5CF6" />}
              iconColor="#8B5CF6"
              title="Privacy protected"
              titleColor="#1F2937"
              description="Your exact location remains private - only general areas are shared"
              delay={400}
            />
          </View>
        </View>
        <View style={styles.actionSection}>
          <TouchableOpacity
            style={styles.ctaButton}
            activeOpacity={0.9}
            onPress={handleEnableLocation}
          >
            <View style={styles.ctaContent}>
              <Text style={styles.ctaText}>Enable location access</Text>
              <View style={styles.ctaIconContainer}>
                <Ionicons name="arrow-forward" size={18} color="#fff" />
              </View>
            </View>
          </TouchableOpacity>
          <TouchableOpacity
            onPress={handleSkip}
            style={styles.skipButton}
            activeOpacity={0.7}
          >
            <Text style={styles.skipText}>Maybe later</Text>
          </TouchableOpacity>
        </View>
      </Animated.View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: "#F8FAFC" },
  backgroundGradient: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    height: height,
  },
  container: { flex: 1, justifyContent: "center", paddingHorizontal: 20 },
  card: {
    backgroundColor: "#fff",
    borderRadius: 24,
    paddingTop: 40,
    paddingHorizontal: 24,
    shadowColor: "#000",
    shadowOpacity: 0.08,
    shadowOffset: { width: 0, height: 12 },
    shadowRadius: 32,
    elevation: 12,
    marginBottom: 24,
  },
  iconSection: { alignItems: "center", marginBottom: 28 },
  iconCircle: {
    width: 80,
    height: 80,
    borderRadius: 40,
    backgroundColor: "#EEF2FF",
    alignItems: "center",
    justifyContent: "center",
    position: "relative",
  },
  iconPulse: {
    shadowColor: "#3B82F6",
    shadowOpacity: 0.3,
    shadowOffset: { width: 0, height: 0 },
    shadowRadius: 20,
    elevation: 8,
  },
  iconInner: {
    width: 56,
    height: 56,
    borderRadius: 28,
    backgroundColor: "#fff",
    alignItems: "center",
    justifyContent: "center",
  },
  textSection: { alignItems: "center", marginBottom: 32 },
  header: {
    fontSize: 26,
    fontWeight: "800",
    color: "#111827",
    textAlign: "center",
    marginBottom: 10,
    lineHeight: 32,
  },
  subHeader: {
    fontSize: 16,
    color: "#6B7280",
    textAlign: "center",
    lineHeight: 24,
    paddingHorizontal: 8,
  },
  infoList: { width: "100%" },
  infoRow: {
    flexDirection: "row",
    alignItems: "flex-start",
    marginBottom: 10,
    paddingHorizontal: 4,
  },
  infoIconContainer: {
    width: 24,
    height: 24, // Fixed for circle!
    borderRadius: 12,
    alignItems: "center",
    justifyContent: "center",
    marginRight: 16,
    marginTop: 2,
  },
  infoTextContainer: { flex: 1, paddingTop: 0 },
  infoTitle: {
    fontSize: 16,
    fontWeight: "700",
    marginBottom: 4,
    lineHeight: 20,
  },
  infoDesc: {
    fontSize: 14,
    color: "#6B7280",
    lineHeight: 20,
  },
  actionSection: { width: "100%", alignItems: "center" },
  ctaButton: {
    width: "100%",
    backgroundColor: "#2563EB",
    paddingVertical: 18,
    paddingHorizontal: 24,
    borderRadius: 16,
    shadowColor: "#2563EB",
    shadowOpacity: 0.3,
    shadowOffset: { width: 0, height: 8 },
    shadowRadius: 16,
    elevation: 8,
  },
  ctaContent: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
  },
  ctaText: {
    color: "#fff",
    fontWeight: "700",
    fontSize: 17,
    letterSpacing: 0.2,
  },
  ctaIconContainer: {
    marginLeft: 12,
    width: 28,
    height: 28,
    borderRadius: 14,
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    alignItems: "center",
    justifyContent: "center",
  },
  skipButton: {
    marginTop: 20,
    paddingVertical: 12,
    paddingHorizontal: 24,
    borderRadius: 12,
  },
  skipText: {
    color: "#9CA3AF",
    fontSize: 16,
    fontWeight: "600",
  },
});

import { Ionicons } from "@expo/vector-icons";
import * as Haptics from "expo-haptics";
import React, { useEffect, useState } from "react";
import {
  Alert,
  Animated,
  Dimensions,
  Image,
  Platform,
  ScrollView,
  StyleSheet,
  Switch,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";
// Use:
import {
  getZipCode,
  setUseAuto,
} from "@/components/account/location/requestLocation";

import authService from "@/database/authService";


  const requestMediaPerms = async () => {
    const { status } = await ImagePicker.requestMediaLibraryPermissionsAsync();
    if (status !== "granted") {
      Alert.alert(
        "Permission needed",
        "We need access to your photos to let you pick images.",
        [{ text: "OK", style: "default" }]
      );
      return false;
    }
    return true;
  };

  const pickImage = async () => {
    const ok = await requestMediaPerms();
    if (!ok) return null;
    const result = await ImagePicker.launchImageLibraryAsync({
      mediaTypes: [MediaType.Image],
      allowsMultipleSelection: false,
      quality: 0.8,
      aspect: [1, 1],
      allowsEditing: true,
    });
    if (result.canceled) return null;
    return result.assets?.[0]?.uri ?? null;
  };

  const changeAvatar = async () => {
    const uri = await pickImage();
    if (!uri) return;
    updateProfile((p) => ({ ...p, avatarUri: uri }));
  };


export function Hero({
    editing,
    profile,
    updateProfile,
    toggleEdit,
    fadeAnim,
    profileStrengthDots,
  }) {
    // Always take first photo as avatar
    const firstPhoto = profile.photos?.[0]?.uri;
  
    // console.log("🔍 Hero: profile.photos:", profile);
    // console.log("🔍 Hero: firstPhoto:", firstPhoto);
  
    const [zip, setZip] = useState(null);
  
    // useEffect(()=>{
    //   console.log("working here", profile)
    // }, [profile])
    useEffect(() => {
      let isMounted = true; // prevent state update after unmount
  
      async function fetchZip() {
        try {
          const z = await getZipCode();
          // console.log("got it");
          if (isMounted) setZip(z);
        } catch (err) {
          console.error("Error getting zipcode:", err);
        }
      }
  
      fetchZip();
  
      return () => {
        isMounted = false; // cleanup
      };
    }, []);
  
    return (
      <Animated.View style={[styles.heroSection, { opacity: fadeAnim }]}>
        <View style={styles.profileHeader}>
          {/* Avatar */}
          <TouchableOpacity
            style={styles.avatarContainer}
            onPress={undefined}
            activeOpacity={0.8}
          >
            <View style={styles.avatarCircle}>
              {firstPhoto ? (
                <Image source={{ uri: firstPhoto }} style={styles.avatarImg} />
              ) : (
                <View style={styles.avatarGradient}>
                  <Text style={styles.avatarText}>{profile.initials}</Text>
                </View>
              )}
            </View>
            <View style={styles.onlineIndicator} />
          </TouchableOpacity>
  
          {/* Info */}
          <View style={styles.profileInfo}>
            {editing.profile ? (
              <>
                <TextInput
                  style={[styles.modernInput, styles.nameInput]}
                  value={profile.name}
                  placeholder="Full name"
                  placeholderTextColor="#9CA3AF"
                  onChangeText={(t) => updateProfile((p) => ({ ...p, name: t }))}
                />
  
                {/* Non-editable location */}
                <View
                  style={[
                    styles.modernInput,
                    styles.locationInput,
                    styles.locationDisplay,
                  ]}
                >
                  <Ionicons name="location" size={16} color="#64748B" />
                  <TextInput
                    style={styles.locationDisplayText}
                    value={zip}
                    placeholder="Enter ZIP"
                    keyboardType="numeric"
                    maxLength={5}
                    onChangeText={(newZip) => {
                      updateProfile((p) => ({
                        ...p,
                        location_sharing: false,
                      }));
                      setUseAuto(false); 
                      setZip(newZip);
                    }}
                  />
                </View>
  
                {/* Location visibility toggle */}
                <View style={styles.locationToggleContainer}>
                  <Text style={styles.toggleLabel}>Show location publicly</Text>
                  <Switch
                    value={profile.location_sharing}
                    onValueChange={async (enabled) => {
                      console.log("inside the  show location", profile)
                      if (Platform.OS === "ios") {
                        Haptics.impactAsync(Haptics.ImpactFeedbackStyle.Light);
                      }
  
                      if (enabled) {
                        try {
                          setUseAuto(true); // ✅ correct usage
  
                          const z = await getZipCode();
                          setZip(z);
                        } catch (err) {
                          console.error(
                            "Error getting zipcode in toggle change:",
                            err
                          );
                        }
                      }
  
                      updateProfile((p) => ({
                        ...p,
                        location_sharing: enabled,
                      }));
                    }}
                    trackColor={{ false: "#D1D5DB", true: "#EF4444" }}
                    thumbColor={profile.location_sharing ? "#FFFFFF" : "#9CA3AF"}
                  />
                </View>
              </>
            ) : (
              <>
                <Text style={styles.name}>{profile.name}</Text>
                <View style={styles.locationContainer}>
                  <Ionicons name="location" size={14} color="#EF4444" />
                  <Text style={styles.location}>{zip}</Text>
                </View>
              </>
            )}
          </View>
  
          {/* Edit / Done button */}
          <TouchableOpacity
            style={[
              styles.modernButton,
              editing.profile && styles.modernButtonActive,
            ]}
            onPress={() => toggleEdit("profile")}
            activeOpacity={0.7}
          >
            <Ionicons
              name={editing.profile ? "checkmark" : "pencil"}
              size={14}
              color={editing.profile ? "#FFFFFF" : "#6B7280"}
            />
            <Text
              style={[
                styles.modernButtonText,
                editing.profile && styles.modernButtonTextActive,
              ]}
            >
              {editing.profile ? "Done" : "Edit"}
            </Text>
          </TouchableOpacity>
        </View>
  
        {/* Profile Strength */}
        {/* <View style={styles.strengthCard}>
          <View style={styles.strengthContent}>
            <View style={styles.strengthLeft}>
              <Text style={styles.strengthNumber}>{profile.strength}%</Text> 
              <Text style={styles.strengthLabel}>Profile Complete</Text>
            </View>
            <View style={styles.strengthIndicator}>
              <View style={styles.strengthRing}>
                <View
                  style={[
                    styles.strengthFill,
                    {
                      transform: [
                        // { rotate: `${(profile.strength / 100) * 360}deg` },
                      ],
                    },
                  ]}
                />
              </View>
              <View style={styles.strengthDots}>
                {profileStrengthDots.map((filled, i) => (
                  <View
                    key={i}
                    style={[
                      styles.strengthDot,
                      filled && styles.strengthDotFilled,
                    ]}
                  />
                ))}
              </View>
            </View>
          </View>
        </View> */}
      </Animated.View>
    );
  }

  const styles = StyleSheet.create({
    /* Hero - Enhanced */
    heroSection: {
        backgroundColor: "#FFFFFF",
        paddingHorizontal: 24,
        paddingTop: 32,
        paddingBottom: 40,
      },
      profileHeader: {
        flexDirection: "row",
        alignItems: "center",
        marginBottom: 32,
      },
      avatarContainer: { marginRight: 20, position: "relative" },
      avatarCircle: {
        width: 88,
        height: 88,
        borderRadius: 44,
        overflow: "hidden",
        shadowColor: "#000",
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.15,
        shadowRadius: 16,
        elevation: 8,
        borderWidth: 3,
        borderColor: "#FFFFFF",
      },
      avatarImg: { width: "100%", height: "100%" },
      avatarGradient: {
        width: "100%",
        height: "100%",
        backgroundColor: "#6366F1",
        justifyContent: "center",
        alignItems: "center",
      },
      avatarText: { fontSize: 26, fontWeight: "800", color: "#FFFFFF" },
      avatarOverlay: {
        position: "absolute",
        bottom: 0,
        right: 0,
        width: 32,
        height: 32,
        borderRadius: 16,
        backgroundColor: "#3B82F6",
        justifyContent: "center",
        alignItems: "center",
        borderWidth: 4,
        borderColor: "#FFFFFF",
        shadowColor: "#3B82F6",
        shadowOffset: { width: 0, height: 2 },
        shadowOpacity: 0.3,
        shadowRadius: 4,
      },
      onlineIndicator: {
        position: "absolute",
        top: 4,
        right: 4,
        width: 20,
        height: 20,
        borderRadius: 10,
        backgroundColor: "#10B981",
        borderWidth: 3,
        borderColor: "#FFFFFF",
      },
      profileInfo: { flex: 1, marginRight: 16 },
      name: {
        fontSize: 20,
        fontWeight: "800",
        color: "#1E293B",
        marginBottom: 6,
        letterSpacing: -0.5,
      },
      locationContainer: { flexDirection: "row", alignItems: "center" },
      location: {
        fontSize: 16,
        color: "#64748B",
        marginLeft: 6,
        fontWeight: "500",
      },
      modernInput: {
        backgroundColor: "#F8FAFC",
        borderWidth: 2,
        borderColor: "#E2E8F0",
        borderRadius: 16,
        paddingHorizontal: 20,
        paddingVertical: 16,
        fontSize: 16,
        color: "#1E293B",
        fontWeight: "600",
      },
      nameInput: {
        fontSize: 14,
        fontWeight: "700",
        marginBottom: 16,
        flexShrink: 1, // let it shrink
        flexWrap: "wrap", // allow wrapping
        minWidth: 0, // needed inside flex row
      },
      locationInput: { fontSize: 16 },
      modernButton: {
        backgroundColor: "#F8FAFC",
        paddingHorizontal: 20,
        paddingVertical: 12,
        borderRadius: 24,
        borderWidth: 2,
        borderColor: "#E2E8F0",
        flexDirection: "row",
        alignItems: "center",
        gap: 8,
        shadowColor: "#000",
        shadowOffset: { width: 0, height: 2 },
        shadowOpacity: 0.05,
        shadowRadius: 4,
        elevation: 2,
      },
      modernButtonActive: {
        backgroundColor: "#3B82F6",
        borderColor: "#3B82F6",
        shadowColor: "#3B82F6",
        shadowOpacity: 0.2,
      },
      modernButtonText: { color: "#64748B", fontSize: 14, fontWeight: "700" },
      modernButtonTextActive: { color: "#FFFFFF" },
      strengthCard: {
        backgroundColor: "#1E293B",
        borderRadius: 24,
        padding: 28,
        shadowColor: "#000",
        shadowOffset: { width: 0, height: 8 },
        shadowOpacity: 0.15,
        shadowRadius: 16,
        elevation: 8,
      },
      strengthContent: {
        flexDirection: "row",
        justifyContent: "space-between",
        alignItems: "center",
      },
      strengthLeft: { flex: 1 },
      strengthNumber: {
        fontSize: 40,
        fontWeight: "900",
        color: "#FFFFFF",
        lineHeight: 44,
      },
      strengthLabel: {
        color: "rgba(255,255,255,0.8)",
        fontSize: 15,
        fontWeight: "600",
        marginTop: 4,
        letterSpacing: 0.5,
      },
      strengthIndicator: { alignItems: "center", gap: 16 },
      strengthRing: {
        width: 60,
        height: 60,
        borderRadius: 30,
        backgroundColor: "rgba(255,255,255,0.1)",
        justifyContent: "center",
        alignItems: "center",
        position: "relative",
        overflow: "hidden",
      },
      strengthFill: {
        position: "absolute",
        top: 0,
        left: 0,
        width: "50%",
        height: "100%",
        backgroundColor: "#10B981",
        transformOrigin: "right center",
      },
      strengthDots: { flexDirection: "row", gap: 8 },
      strengthDot: {
        width: 8,
        height: 8,
        borderRadius: 4,
        backgroundColor: "rgba(255,255,255,0.3)",
      },
      strengthDotFilled: { backgroundColor: "#FFFFFF" },
    })
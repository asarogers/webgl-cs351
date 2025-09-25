// AccountScreen.tsx — Simplified, no caching, single fetch

import { Ionicons } from "@expo/vector-icons";
import * as Haptics from "expo-haptics";
import * as Location from "expo-location";
import React, { useEffect, useMemo, useState } from "react";
import * as ImagePicker from "expo-image-picker";
import {
  Alert,
  Animated,
  Platform,
  ScrollView,
  StatusBar,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from "react-native";
import { INTERESTS, QUIZ } from "../../../../components/Interests_lifestyle";
import authService from "../../../../database/authService";
import ImagePickerComponent from "../../../../components/account/ImagePickerComponent";

// Extracted UI components
import {
  Hero,
  StickyHeader,
} from "../../../../components/account/_TopComponents";

/* ============================================================================
 * CONSTANTS & DEFAULTS
 * ========================================================================== */

// Helper: flatten all available interests from INTERESTS
const getAllInterests = () => {
  const allInterests = [];
  INTERESTS.groupsOrder.forEach((groupKey) => {
    const group = INTERESTS.groups[groupKey];
    group.optionsOrder.forEach((optionKey) => {
      allInterests.push(group.options[optionKey]);
    });
  });
  return allInterests;
};

const defaultProfile = {
  profile: {
    about: "Updating this thing here",
    avatarUri: null,
    basic: {
      company: "",
      education: "",
    },
    housing: {
      budget: "",
      lease: "",
      moveIn: "",
      roomSize: "",
    },
    initials: "AR",
    interests: [
      "Baking",
      "Coffee",
      "Hosts Occasionally",
      "Quiet Evenings",
      "Clean & Organized",
      "DIY/Handy",
    ],
    lifestyle: {
      dogOwner: false,
      drinks: false,
      nonSmoker: true,
      petFriendly: false,
    },
    location: "ZIP 60645",
    name: "Asa Rogers",
    photos: [],
    rawZipcode: "60645",
    strength: 29,
  },
  raw: {
    about: "Updating this thing here",
    budget_max: null,
    budget_min: null,
    company: null,
    dish_washing: "dishwasher",
    education: null,
    email: "cyberasasoftware@gmail.com",
    first_name: "Asa",
    friends_over: "party",
    id: "592f0cd2-3644-4328-950e-7c61927dc2fd",
    interests: [
      "Baking",
      "Coffee",
      "Hosts Occasionally",
      "Quiet Evenings",
      "Clean & Organized",
      "DIY/Handy",
    ],
    last_name: "Rogers",
    lease_duration: null,
    location_sharing: true,
    move_in_selection: null,
    pet_situation: "none",
    photos: [],
    sleep_schedule: "flexi",
    weekend_vibe: null,
    zipcode: "60645",
    zone_drawn: null,
  },
  success: true,
};

/* ============================================================================
 * PARSING HELPERS
 * ========================================================================== */

const parseBudgetRangeLabel = (label) => {
  if (!label)
    return [
      defaultProfile.basics.budget_range[0],
      defaultProfile.basics.budget_range[1],
    ];
  const clean = String(label).replace(/[\$,]/g, "");
  const mRange = clean.match(/(\d+)\s*[–-]\s*(\d+)/);
  if (mRange) return [parseInt(mRange[1], 10), parseInt(mRange[2], 10)];
  const mUpTo = clean.match(/up to\s*(\d+)/i);
  if (mUpTo) return [0, parseInt(mUpTo[1], 10)];
  const mPlus = clean.match(/(\d+)\s*\+/);
  if (mPlus) return [parseInt(mPlus[1], 10), 0];
  const lone = clean.match(/^\d+$/);
  if (lone) return [parseInt(clean, 10), 0];
  return [
    defaultProfile.basics.budget_range[0],
    defaultProfile.basics.budget_range[1],
  ];
};

/* ============================================================================
 * UPLOAD HELPERS
 * ========================================================================== */

const isLocalUri = (uri) => !!uri && uri.startsWith("file://");

const uploadAndGetUrl = async (uri) => {
  try {
    if (!uri) throw new Error("No URI provided");

    if (typeof authService?.uploadImageFromUI === "function") {
      console.log("📤 Uploading image:", uri.substring(0, 50) + "...");

      const uploaded = await authService.uploadImageFromUI(uri);

      if (uploaded?.url) {
        console.log("✅ Upload successful");
        return uploaded.url;
      } else if (uploaded?.error) {
        throw new Error(uploaded.error);
      }
    }

    // Fallback: return original URI if upload service not available
    console.log("⚠️ Upload service not available, using original URI");
    return uri;
  } catch (error) {
    console.error("Upload error:", error);
    throw error; // Re-throw to be handled by caller
  }
};

/* ============================================================================
 * MAIN COMPONENT
 * ========================================================================== */

const AccountScreen = () => {
  const [profile, setProfile] = useState(defaultProfile);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState(null);
  const [dirty, setDirty] = useState(false);
  const [initialLoadComplete, setInitialLoadComplete] = useState(false);

  const [editing, setEditing] = useState({
    profile: false,
    photos: false,
    about: false,
    interests: false,
    substances: false,
    lifestyle: false,
    basics: false,
    pets: false,
    amenities: false,
  });

  const [fadeAnim] = useState(new Animated.Value(1));

  // Single fetch on mount
  useEffect(() => {
    (async () => {
      try {
        setDirty(false);

        const res = await authService.getAccountProfileForUI();
        let profileData = res.profile;

        // ✅ Fetch Supabase photos for this user
        const photoRes = await authService.getUserPhotos({
          signed: true,
        });

        if (photoRes.success && photoRes.photos?.length > 0) {
          // Map into {id, uri}
          const mapped = photoRes.photos.map((url, i) => ({
            id: `remote_${i}`,
            uri: url,
          }));

          profileData = {
            ...profileData,
            photos: mapped,
          };
        }

        // console.log("came from load", profileData)

        setProfile(profileData);
        setInitialLoadComplete(true);

        if (res?.success && res.profile) {
          // Show location sharing prompt if not enabled
          if (res.profile.location_sharing === false) {
            Alert.alert(
              "Share Your Location?",
              "Would you like to share your location so roommates can discover you more easily?",
              [
                {
                  text: "Not now",
                  style: "cancel",
                },
                {
                  text: "Yes, share",
                  onPress: async () => {
                    try {
                      const { status } =
                        await Location.requestForegroundPermissionsAsync();
                      if (status !== "granted") {
                        Alert.alert(
                          "Permission denied",
                          "Enable location in settings later."
                        );
                        return;
                      }

                      const { coords } = await Location.getCurrentPositionAsync(
                        {}
                      );
                      const places = await Location.reverseGeocodeAsync(coords);
                      const zip = places[0]?.postalCode || null;

                      // Update location sharing
                      const updatedProfile = {
                        ...res.profile,
                        location_sharing: true,
                        location: zip,
                      };

                      await saveProfile(updatedProfile);
                    } catch (e) {
                      console.error("Failed to enable location sharing:", e);
                    }
                  },
                },
              ]
            );
          }
        } else if (res?.error) {
          setError(res.error);
        }
      } catch (e) {
        setError(e?.message || "Failed to load profile");
      } finally {
        setLoading(false);
      }
    })();
  }, []);

  /* ==========================================================================
   * SAVE LOGIC
   * ======================================================================== */

  const saveProfile = async (updatedProfile = profile) => {
    try {
      setSaving(true);
      setError(null);

      console.log(
        "🔍 saveProfile: saving profile with photos:",
        updatedProfile.photos
      );

      const res = await authService.updateAccountProfileFromUI(updatedProfile);

      if (!res?.success) {
        throw new Error(res?.error || "Failed to save");
      }

      console.log(
        "🔍 saveProfile: received back from server:",
        res.profile?.photos
      );

      setProfile(res.profile);
      setDirty(false);
    } catch (e) {
      setError(e?.message || "Save failed.");
    } finally {
      setSaving(false);
    }
  };

  const updateProfile = (updater) => {
    // console.log("callled");
    if (initialLoadComplete) {
      setDirty(true);
    }
    setProfile((prev) => {
      const next = typeof updater === "function" ? updater(prev) : updater;
      return next;
    });
  };

  const toggleEdit = (key) => {
    if (Platform.OS === "ios")
      Haptics.impactAsync(Haptics.ImpactFeedbackStyle.Light);
    setEditing((e) => ({ ...e, [key]: !e[key] }));
  };

  /* ==========================================================================
   * MEDIA SELECTION
   * ======================================================================== */



  /* ==========================================================================
   * DISPLAY UTILITIES
   * ======================================================================== */

  const profileStrengthDots = useMemo(() => {
    const filled = Math.round((profile.strength / 100) * 5);
    return Array.from({ length: 5 }, (_, i) => i < filled);
  }, [profile.strength]);

  const formatBudgetRange = (range) => `$${range[0]}-$${range[1]}`;
  const formatDuration = (months) => {
    if (!months) return "Open-ended";
    if (months < 12) return `${months} months`;
    const years = Math.floor(months / 12);
    const remainingMonths = months % 12;
    if (remainingMonths === 0) return `${years} year${years > 1 ? "s" : ""}`;
    return `${years}.${
      Math.round(remainingMonths / 6) * 6 === 6 ? "5" : "0"
    } years`;
  };

  const getOptionLabel = (sectionKey, questionKey, optionKey) => {
    const section = QUIZ.sections[sectionKey];
    if (!section) return optionKey;
    const question = section.questions[questionKey];
    if (!question || !question.options) return optionKey;
    const option = question.options[optionKey];
    return option?.label || optionKey;
  };

  /* ==========================================================================
   * EDITOR COMPONENTS
   * ======================================================================== */

  const MultiSelectEditor = ({
    title,
    options,
    selected,
    onChange,
    exclusiveWith = [],
    sectionKey,
    questionKey,
  }) => (
    <View style={styles.multiSelectContainer}>
      <Text style={styles.multiSelectTitle}>{title}</Text>
      <View style={styles.selectableTagsContainer}>
        {options.map((optionKey) => {
          const isSelected = selected.includes(optionKey);
          const label = getOptionLabel(sectionKey, questionKey, optionKey);
          return (
            <TouchableOpacity
              key={optionKey}
              style={[styles.choiceTag, isSelected && styles.choiceTagSelected]}
              onPress={() => {
                if (Platform.OS === "ios") Haptics.selectionAsync();
                let nextSelected = isSelected
                  ? selected.filter((i) => i !== optionKey)
                  : [...selected, optionKey];

                exclusiveWith.forEach(([exclusive, conflicting]) => {
                  if (optionKey === exclusive) {
                    nextSelected = nextSelected.filter(
                      (i) => !conflicting.includes(i)
                    );
                  } else if (conflicting.includes(optionKey)) {
                    nextSelected = nextSelected.filter((i) => i !== exclusive);
                  }
                });

                onChange(nextSelected);
              }}
              activeOpacity={0.8}
            >
              <Text
                style={[
                  styles.choiceTagText,
                  isSelected && styles.choiceTagTextSelected,
                ]}
              >
                {label}
              </Text>
              {isSelected && (
                <Ionicons name="checkmark" size={14} color="#FFFFFF" />
              )}
            </TouchableOpacity>
          );
        })}
      </View>
    </View>
  );

  const SingleSelectEditor = ({
    title,
    options,
    selected,
    onChange,
    sectionKey,
    questionKey,
  }) => (
    <View style={styles.singleSelectContainer}>
      <Text style={styles.singleSelectTitle}>{title}</Text>
      <View style={styles.selectOptionsContainer}>
        {options.map((optionKey) => {
          const isSelected = selected === optionKey;
          const label = getOptionLabel(sectionKey, questionKey, optionKey);
          return (
            <TouchableOpacity
              key={optionKey}
              style={[
                styles.selectOption,
                isSelected && styles.selectOptionSelected,
              ]}
              onPress={() => {
                if (Platform.OS === "ios") Haptics.selectionAsync();
                onChange(optionKey);
              }}
              activeOpacity={0.8}
            >
              <Text
                style={[
                  styles.selectOptionText,
                  isSelected && styles.selectOptionTextSelected,
                ]}
              >
                {label}
              </Text>
              {isSelected && (
                <Ionicons name="checkmark-circle" size={20} color="#3B82F6" />
              )}
            </TouchableOpacity>
          );
        })}
      </View>
    </View>
  );

  const ScaleEditor = ({ title, value, onChange, leftAnchor, rightAnchor }) => (
    <View style={styles.scaleEditorContainer}>
      <Text style={styles.scaleEditorTitle}>{title}</Text>
      <View style={styles.scaleEditorContent}>
        <Text style={styles.scaleAnchor}>{leftAnchor}</Text>
        <View style={styles.scaleDotsContainer}>
          {[1, 2, 3, 4, 5].map((level) => (
            <TouchableOpacity
              key={level}
              style={[
                styles.scaleEditorDot,
                value >= level && styles.scaleEditorDotActive,
              ]}
              onPress={() => {
                if (Platform.OS === "ios") Haptics.selectionAsync();
                onChange(level);
              }}
              activeOpacity={0.8}
            >
              <Text style={styles.scaleEditorDotText}>{level}</Text>
            </TouchableOpacity>
          ))}
        </View>
        <Text style={styles.scaleAnchor}>{rightAnchor}</Text>
      </View>
    </View>
  );

  /* ==========================================================================
   * RENDER
   * ======================================================================== */

  if (loading) {
    return (
      <View
        style={[
          styles.container,
          { alignItems: "center", justifyContent: "center" },
        ]}
      >
        <StatusBar barStyle="dark-content" backgroundColor="#FFFFFF" />
        <Text style={{ color: "#6B7280" }}>Loading profile…</Text>
      </View>
    );
  }

  return (
    <>
      <StatusBar barStyle="dark-content" backgroundColor="#FFEDD5" />

      {/* Sticky header */}
      <StickyHeader
        saving={saving}
        dirty={dirty}
        syncNow={() => saveProfile()}
        editing={editing}
        setDirty={setDirty}
      />

      <ScrollView
        style={styles.container}
        showsVerticalScrollIndicator={false}
        contentContainerStyle={{ paddingBottom: 40, paddingTop: 8 }}
      >
        <Hero
          editing={editing}
          profile={profile}
          updateProfile={updateProfile}
          toggleEdit={toggleEdit}
          changeAvatar={changeAvatar}
          fadeAnim={fadeAnim}
          profileStrengthDots={profileStrengthDots}
        />
        <ImagePickerComponent
          editing={editing}
          profile={profile}
          toggleEdit={toggleEdit}
          initialLoadComplete={initialLoadComplete}
          onPhotosChange={(photos) => {
            console.log(
              "🔍 AccountScreen: onPhotosChange called with:",
              photos
            );
            updateProfile((p) => {
              const updated = { ...p, photos };
              console.log(
                "🔍 AccountScreen: updating profile photos to:",
                updated.photos
              );
              return updated;
            });
          }}
        />

        {/*
        <AboutSection
          editing={editing}
          profile={profile}
          updateProfile={updateProfile}
          toggleEdit={toggleEdit}
        />

        <Interests
          editing={editing}
          profile={profile}
          updateProfile={updateProfile}
          toggleEdit={toggleEdit}
          getAllInterests={getAllInterests}
        />

        <Basics
          editing={editing}
          profile={profile}
          updateProfile={updateProfile}
          toggleEdit={toggleEdit}
          QUIZ={QUIZ}
          SingleSelectEditor={SingleSelectEditor}
          getOptionLabel={getOptionLabel}
          formatBudgetRange={formatBudgetRange}
          formatDuration={formatDuration}
        />

        <Lifestyle
          editing={editing}
          profile={profile}
          updateProfile={updateProfile}
          toggleEdit={toggleEdit}
          QUIZ={QUIZ}
          SingleSelectEditor={SingleSelectEditor}
          ScaleEditor={ScaleEditor}
          getOptionLabel={getOptionLabel}
        />

        <Substances
          editing={editing}
          profile={profile}
          updateProfile={updateProfile}
          toggleEdit={toggleEdit}
          QUIZ={QUIZ}
          SingleSelectEditor={SingleSelectEditor}
          getOptionLabel={getOptionLabel}
        />

        <Pets
          editing={editing}
          profile={profile}
          updateProfile={updateProfile}
          toggleEdit={toggleEdit}
          QUIZ={QUIZ}
          MultiSelectEditor={MultiSelectEditor}
          getOptionLabel={getOptionLabel}
        />

        <Amenities
          editing={editing}
          profile={profile}
          updateProfile={updateProfile}
          toggleEdit={toggleEdit}
          QUIZ={QUIZ}
          SingleSelectEditor={SingleSelectEditor}
          MultiSelectEditor={MultiSelectEditor}
          getOptionLabel={getOptionLabel}
        />


        <TouchableOpacity
          style={[styles.modernCTAButton, saving && styles.savingButton]}
          onPress={() => saveProfile()}
          disabled={saving}
          activeOpacity={0.9}
        >
          <View style={styles.ctaButtonContent}>
            <Ionicons 
              name={saving ? "sync" : "save-outline"} 
              size={20} 
              color="#FFFFFF" 
            />
            <Text style={styles.ctaButtonText}>
              {saving ? "Saving..." : "Save Profile"}
            </Text>
          </View>
        </TouchableOpacity>


        <TouchableOpacity
          style={styles.secondaryCTAButton}
          onPress={() => {
            Alert.alert(
              "Preview Profile",
              "This would show your profile as others see it."
            );
          }}
          activeOpacity={0.9}
        >
          <View style={styles.ctaButtonContent}>
            <Ionicons name="eye-outline" size={20} color="#6366F1" />
            <Text style={styles.secondaryButtonText}>Preview Profile</Text>
          </View>
        </TouchableOpacity>

        {error && (
          <View style={styles.errorContainer}>
            <Text style={styles.errorText}>{error}</Text>
            <TouchableOpacity
              onPress={() => setError(null)}
              style={styles.dismissError}
            >
              <Ionicons name="close" size={16} color="#EF4444" />
            </TouchableOpacity>
          </View> 
        )}*/}
      </ScrollView>
    </>
  );
};

/* ============================================================================
 * STYLES
 * ========================================================================== */

const styles = StyleSheet.create({
  /* Screen container */
  container: {
    flex: 1,
    backgroundColor: "#F8FAFC",
  },

  /* CTA at bottom */
  modernCTAButton: {
    backgroundColor: "#6366F1",
    marginHorizontal: 24,
    marginTop: 24,
    borderRadius: 16,
    paddingVertical: 16,
    shadowColor: "#6366F1",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.3,
    shadowRadius: 12,
    elevation: 6,
  },
  ctaButtonContent: {
    flexDirection: "row",
    justifyContent: "center",
    alignItems: "center",
    gap: 8,
  },
  ctaButtonText: {
    color: "#FFFFFF",
    fontSize: 16,
    fontWeight: "700",
    letterSpacing: 0.5,
  },

  /* Multi-select tags */
  selectableTagsContainer: {
    flexDirection: "row",
    flexWrap: "wrap",
    gap: 8,
  },
  choiceTag: {
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
    paddingHorizontal: 14,
    paddingVertical: 8,
    borderRadius: 18,
    borderWidth: 1.5,
    borderColor: "#E5E7EB",
    backgroundColor: "#FFFFFF",
  },
  choiceTagSelected: {
    backgroundColor: "#3B82F6",
    borderColor: "#3B82F6",
    shadowColor: "#3B82F6",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.2,
    shadowRadius: 4,
    elevation: 2,
  },
  choiceTagText: {
    color: "#374151",
    fontSize: 13,
    fontWeight: "600",
  },
  choiceTagTextSelected: {
    color: "#FFFFFF",
  },

  /* MultiSelectEditor */
  multiSelectContainer: {
    marginTop: 16,
    gap: 10,
  },
  multiSelectTitle: {
    fontSize: 14,
    fontWeight: "700",
    color: "#1F2937",
    marginBottom: 4,
  },

  /* SingleSelectEditor */
  singleSelectContainer: {
    marginTop: 16,
    gap: 10,
  },
  singleSelectTitle: {
    fontSize: 14,
    fontWeight: "700",
    color: "#1F2937",
    marginBottom: 4,
  },
  selectOptionsContainer: {
    gap: 8,
  },
  selectOption: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    paddingVertical: 10,
    paddingHorizontal: 14,
    borderRadius: 12,
    borderWidth: 1.5,
    borderColor: "#E5E7EB",
    backgroundColor: "#FFFFFF",
  },
  selectOptionSelected: {
    borderColor: "#3B82F6",
    backgroundColor: "#EFF6FF",
  },
  selectOptionText: {
    color: "#374151",
    fontSize: 14,
    fontWeight: "600",
  },
  selectOptionTextSelected: {
    color: "#1D4ED8",
  },

  /* ScaleEditor */
  scaleEditorContainer: {
    marginTop: 16,
    gap: 10,
  },
  scaleEditorTitle: {
    fontSize: 14,
    fontWeight: "700",
    color: "#1F2937",
  },
  scaleEditorContent: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    gap: 12,
  },
  scaleAnchor: {
    fontSize: 12,
    color: "#6B7280",
    width: 72,
    textAlign: "center",
  },
  scaleDotsContainer: {
    flexDirection: "row",
    gap: 8,
    flex: 1,
    justifyContent: "center",
  },
  scaleEditorDot: {
    minWidth: 36,
    paddingVertical: 8,
    paddingHorizontal: 10,
    borderRadius: 10,
    borderWidth: 1.5,
    borderColor: "#E5E7EB",
    backgroundColor: "#FFFFFF",
    alignItems: "center",
  },
  scaleEditorDotActive: {
    borderColor: "#3B82F6",
    backgroundColor: "#EFF6FF",
  },
  scaleEditorDotText: {
    fontSize: 12,
    fontWeight: "700",
    color: "#374151",
  },
});

export default AccountScreen;

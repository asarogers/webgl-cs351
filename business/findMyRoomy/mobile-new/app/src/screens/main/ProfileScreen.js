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
import { StickyHeader } from "../../../../components/account/_TopComponents";
import { Hero } from "../../../../components/account/Hero";

export default function ProfileScreen() {
  const [profile, setProfile] = useState(null);
  const [loading, setLoading] = useState(null);
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

  useEffect(() => {
    (async () => {
      try {
        setDirty(false);

        const res = await authService.getAccountProfileForUI();
        let profileData = res.profile;

        if (profileData.photos?.length > 0) {
          // Map into {id, uri}
          const mapped = profileData.photos.map((url, i) => ({
            id: `remote_${i}`,
            uri: url,
          }));

          profileData = {
            ...profileData,
            photos: mapped,
          };
        }

        setProfile(profileData);
      } catch (err) {
        console.log("Error on mount in profile screen", err);
      }
    })();
  }, []);

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
    // if (initialLoadComplete) {
    setDirty(true);
    // }
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


  // if (loading) {
  //   return (
  //     <View
  //       style={[
  //         styles.container,
  //         { alignItems: "center", justifyContent: "center" },
  //       ]}
  //     >
  //       <StatusBar barStyle="dark-content" backgroundColor="#FFFFFF" />
  //       <Text style={{ color: "#6B7280" }}>Loading profile…</Text>
  //     </View>
  //   );
  // }

  return (
    <>
      {/* <StatusBar barStyle="dark-content" backgroundColor="#FFEDD5" /> */}
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
        {profile ? (
          <>
            <Hero
              editing={editing}
              profile={profile}
              updateProfile={updateProfile}
              toggleEdit={toggleEdit}
              fadeAnim={fadeAnim}
            />

            <ImagePickerComponent
              editing={editing}
              profile={profile}
              toggleEdit={toggleEdit}
              initialLoadComplete={initialLoadComplete}
              setProfile = {setProfile}
              setDirty= {setDirty}

            />
          </>
        ) : (
          <Text style={{ color: "#6B7280", textAlign: "center" }}>
            Loading profile…
          </Text>
        )}
      </ScrollView>
    </>
  );
}

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

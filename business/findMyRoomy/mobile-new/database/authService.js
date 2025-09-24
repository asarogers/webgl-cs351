// authService.js
import { supabase } from "./supabaseClient";
import * as AuthSession from "expo-auth-session";
import * as WebBrowser from "expo-web-browser";
import * as Linking from "expo-linking";
export const STORAGE_BUCKET = "user-photos";
import { uriToBlob, uriToUint8Array } from "@/utils/uriToBlob";

WebBrowser.maybeCompleteAuthSession();

class AuthService {
  constructor() {
    // Create a custom redirect URL for your app
    this.redirectUrl = Linking.createURL("/auth");
  }

  parseFullName(fullName) {
    if (!fullName) return { firstName: "", lastName: "" };
    const parts = fullName.trim().split(/\s+/);
    const firstName = parts.shift() || "";
    const lastName = parts.length ? parts.join(" ") : "";
    return { firstName, lastName };
  }

  async ensureUserRowAndDefaultOnboarding(authUser) {
    const { id, email, user_metadata } = authUser;
    const fullName =
      user_metadata?.full_name ||
      user_metadata?.name ||
      user_metadata?.display_name ||
      "";
    const { firstName, lastName } = this.parseFullName(fullName);

    const { error } = await supabase.from("users").upsert(
      [
        {
          id, // must be PK/unique in `users`
          email,
          first_name: firstName,
          last_name: lastName,
          onboarding: "not_started",
          updated_at: new Date().toISOString(),
        },
      ],
      { onConflict: "id" }
    );
    if (error) throw error;
  }

  async saveUserIntent(intent) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      // Update the user's intent field in the users table
      const { error } = await supabase
        .from("users")
        .update({ intent }) // use 'user_intent' if that's your column name
        .eq("id", user.id);

      if (error) throw error;
      return { success: true };
    } catch (error) {
      console.error("Save User Intent Error:", error);
      return {
        success: false,
        error: error.message || "Failed to save intent",
      };
    }
  }

  // Updated signInWithGoogle method - Fixed version
  async signInWithGoogle() {
    try {
      // Step 1: Get redirect URI for Expo AuthSession
      const redirectUri = AuthSession.makeRedirectUri({
        useProxy: true,
        preferLocalhost: true,
      });

      // Step 2: Start Supabase OAuth flow
      const { data, error } = await supabase.auth.signInWithOAuth({
        provider: "google",
        options: {
          redirectTo: redirectUri,
          queryParams: {
            access_type: "offline",
            prompt: "consent",
          },
        },
      });
      if (error) throw error;
      if (!data?.url) throw new Error("No OAuth URL received from Supabase");

      // Step 3: Open AuthSession for Google
      const result = await WebBrowser.openAuthSessionAsync(
        data.url,
        redirectUri
      );

      if (result.type === "success" && result.url) {
        // Parse tokens from callback URL
        const url = new URL(result.url);
        let params;
        if (url.hash) {
          params = new URLSearchParams(url.hash.substring(1));
        } else if (url.search) {
          params = new URLSearchParams(url.search);
        } else {
          throw new Error("No parameters found in callback URL");
        }
        const accessToken = params.get("access_token");
        const refreshToken = params.get("refresh_token");
        if (!accessToken) throw new Error("No access token received");

        // Step 4: Set session in Supabase
        const { data: sessionData, error: sessionError } =
          await supabase.auth.setSession({
            access_token: accessToken,
            refresh_token: refreshToken,
          });
        if (sessionError) throw sessionError;

        // Step 5: Get the authenticated user
        const { data: userData, error: userError } =
          await supabase.auth.getUser();
        if (userError) throw userError;
        const user = userData.user;

        // Step 6: Ensure user exists in your users table
        if (user) {
          const { data: userRow } = await supabase
            .from("users")
            .select("id")
            .eq("id", user.id)
            .single();

          if (!userRow) {
            // Helper function to parse full name into first and last name
            const parseFullName = (fullName) => {
              if (!fullName) return { firstName: "", lastName: "" };
              const nameParts = fullName.trim().split(" ");
              const firstName = nameParts[0] || "";
              const lastName =
                nameParts.length > 1 ? nameParts.slice(1).join(" ") : "";
              return { firstName, lastName };
            };

            // Try different possible metadata fields that Google might provide
            const fullName =
              user.user_metadata?.full_name ||
              user.user_metadata?.name ||
              user.user_metadata?.display_name ||
              "";

            const { firstName, lastName } = parseFullName(fullName);

            // Debug logging to see what metadata is available
            // console.log("Google OAuth user metadata:", user.user_metadata);
            // console.log("Parsed name - First:", firstName, "Last:", lastName);

            await supabase.from("users").insert([
              {
                id: user.id,
                email: user.email,
                first_name: firstName,
                last_name: lastName,
                onboarding: "not_started",
              },
            ]);
          }
        }
        return {
          success: true,
          user,
          session: sessionData.session,
        };
      } else if (result.type === "cancel") {
        return { success: false, error: "Google sign-in was cancelled" };
      } else {
        throw new Error(`OAuth flow failed: ${result.type}`);
      }
    } catch (error) {
      console.error("Google Sign-In Error:", error);
      return {
        success: false,
        error: error.message || "Google sign-in failed. Please try again.",
      };
    }
  }

  // Email/Password Sign Up
  async signUpWithEmail(email, password, firstName, lastName) {
    try {
      const { data, error } = await supabase.auth.signUp({
        email,
        password,
        options: {
          data: {
            first_name: firstName,
            last_name: lastName,
            display_name: `${firstName} ${lastName}`,
          },
        },
      });
      if (error) throw error;
      const userId = data.user?.id;

      // Step 2: Save the user to your users table
      if (userId) {
        const { error: dbError } = await supabase.from("users").insert([
          {
            id: userId, // use the same uuid from auth
            first_name: firstName,
            last_name: lastName,
            email,
            onboarding: "not_started",
          },
        ]);
        if (dbError) throw dbError;
      }

      return {
        success: true,
        user: data.user,
        session: data.session,
        needsVerification: !data.session,
      };
    } catch (error) {
      console.error("Email Sign-Up Error:", error);
      return { success: false, error: error.message || "Sign-up failed" };
    }
  }

  async signOut() {
    try {
      const { error } = await supabase.auth.signOut();
      if (error) throw error;
      return { success: true };
    } catch (error) {
      console.error("Sign-Out Error:", error);
      return { success: false, error: error.message || "Sign-out failed" };
    }
  }

  async getCurrentUser() {
    try {
      const {
        data: { user },
        error,
      } = await supabase.auth.getUser();
      if (error) throw error;
      return { success: true, user };
    } catch (error) {
      console.error("Get Current User Error:", error);
      return {
        success: false,
        error: error.message || "Failed to get current user",
      };
    }
  }

  // Email/Password Sign In
  async signInWithEmail(email, password) {
    try {
      // 1. Try to sign in using Supabase Auth
      const { data, error } = await supabase.auth.signInWithPassword({
        email,
        password,
      });
      if (error) throw error;
      const user = data.user;
      const session = data.session;

      // 2. Make sure the user exists in your custom users table
      if (user) {
        // See if we have a profile already
        const { data: userRow } = await supabase
          .from("users")
          .select("id")
          .eq("id", user.id)
          .single();

        // If not, insert a default row (you might only have email available)
        if (!userRow) {
          await supabase.from("users").insert([
            {
              id: user.id,
              email: user.email,
              first_name: user.user_metadata?.first_name || "",
              last_name: user.user_metadata?.last_name || "",
              onboarding: "not_started",
            },
          ]);
        }
      }

      // 3. Lookup onboarding step for this user!
      let onboarding = await this.getOnboardingStep(user.id);

      return {
        success: true,
        user,
        session,
        onboarding,
      };
    } catch (error) {
      console.error("Email Sign-In Error:", error);
      return {
        success: false,
        error:
          error.message ||
          "Sign-in failed. Please check your email and password.",
      };
    }
  }

  async getZip() {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError || !user)
        throw userError || new Error("No authenticated user");
      const { data, error } = await supabase
        .from("users")
        .select("zipcode")
        .eq("id", user.id)
        .single();
      if (error || !data)
        throw error || new Error("Failed to fetch visibility");
      return data.is_visible; // true = visible, false = hidden
    } catch (err) {
      console.error("getProfileVisibility Error:", err);
      return null;
    }
  }


  async getProfileVisibility() {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError || !user)
        throw userError || new Error("No authenticated user");
      const { data, error } = await supabase
        .from("users")
        .select("is_visible")
        .eq("id", user.id)
        .single();
      if (error || !data)
        throw error || new Error("Failed to fetch visibility");
      return data.is_visible; // true = visible, false = hidden
    } catch (err) {
      console.error("getProfileVisibility Error:", err);
      return null;
    }
  }
  async updateAccountProfilePatch(patch, options = {}) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      // Add timestamp to all updates
      const dataToUpdate = {
        ...patch,
        updated_at: new Date().toISOString(),
      };

      const { data, error } = await supabase
        .from("users")
        .update(dataToUpdate)
        .eq("id", user.id)
        .select(
          `
          id, first_name, last_name, email, avatarUri, initials, name, location,
          about, interests, photos, strength,
          substances, lifestyle, basics, pets, amenities,
          education, company, weekend_vibe, sleep_schedule, dish_washing, 
          friends_over, pet_situation, budget_min, budget_max,
          move_in_selection, lease_duration, zone_drawn, location_sharing,
          verification_level, is_visible, intent, onboarding
        `
        )
        .single();

      if (error) throw error;

      return { success: true, data };
    } catch (err) {
      console.error("updateAccountProfilePatch Error:", err);
      return {
        success: false,
        error: err.message || "Failed to update account profile",
      };
    }
  }

  async updateAccountProfileFull(profileData, options = {}) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      // Convert the AccountScreen profile format to database format
      const dbUpdates = this.convertAccountProfileToDbFormat(profileData);

      // Use the same update method but with full profile data
      return await this.updateAccountProfilePatch(dbUpdates, {
        ...options,
        isFull: true,
      });
    } catch (err) {
      console.error("updateAccountProfileFull Error:", err);
      return {
        success: false,
        error: err.message || "Failed to update full account profile",
      };
    }
  }

  convertAccountProfileToDbFormat(profile) {
    const dbFormat = {};

    // Basic profile info
    if (profile.avatarUri !== undefined) dbFormat.avatarUri = profile.avatarUri;
    if (profile.about !== undefined) dbFormat.about = profile.about;
    if (profile.interests !== undefined)
      dbFormat.interests = Array.isArray(profile.interests)
        ? profile.interests
        : [];
    if (profile.photos !== undefined) {
      dbFormat.photos = Array.isArray(profile.photos)
        ? profile.photos
            .map((p) => (typeof p === "string" ? p : p?.uri))
            .filter(Boolean)
        : [];
    }

    // Substances
    if (profile.substances) {
      if (profile.substances.smoking_policy !== undefined)
        dbFormat.smoking_policy = profile.substances.smoking_policy;
      if (profile.substances.alcohol_use !== undefined)
        dbFormat.alcohol_use = profile.substances.alcohol_use;
      if (profile.substances.cannabis_policy !== undefined)
        dbFormat.cannabis_policy = profile.substances.cannabis_policy;
      if (profile.substances.substance_free_preference !== undefined)
        dbFormat.substance_free_preference =
          profile.substances.substance_free_preference;
    }

    // Lifestyle
    if (profile.lifestyle) {
      if (profile.lifestyle.cleanliness_level !== undefined)
        dbFormat.cleanliness_level = profile.lifestyle.cleanliness_level;
      if (profile.lifestyle.cleaning_frequency !== undefined)
        dbFormat.cleaning_frequency = profile.lifestyle.cleaning_frequency;
      if (profile.lifestyle.home_presence !== undefined)
        dbFormat.home_presence = profile.lifestyle.home_presence;
      if (profile.lifestyle.sleep_rhythm !== undefined)
        dbFormat.sleep_rhythm = profile.lifestyle.sleep_rhythm;
      if (profile.lifestyle.guest_frequency !== undefined)
        dbFormat.guest_frequency = profile.lifestyle.guest_frequency;
      if (profile.lifestyle.overnight_guests !== undefined)
        dbFormat.overnight_guests = profile.lifestyle.overnight_guests;
      if (profile.lifestyle.noise_tolerance !== undefined)
        dbFormat.noise_tolerance = profile.lifestyle.noise_tolerance;
    }

    // Basics/Housing
    if (profile.basics) {
      if (profile.basics.budget_range !== undefined) {
        dbFormat.budget_min = profile.basics.budget_range[0] || null;
        dbFormat.budget_max = profile.basics.budget_range[1] || null;
      }
      if (profile.basics.move_in_timeline !== undefined)
        dbFormat.move_in_selection = profile.basics.move_in_timeline;
      if (profile.basics.minimum_stay !== undefined)
        dbFormat.minimum_stay = profile.basics.minimum_stay;
      if (profile.basics.maximum_stay !== undefined)
        dbFormat.maximum_stay = profile.basics.maximum_stay;
      if (profile.basics.lease_duration !== undefined)
        dbFormat.lease_duration = profile.basics.lease_duration;
      if (profile.basics.room_type !== undefined)
        dbFormat.room_type = profile.basics.room_type;
    }

    // Pets
    if (profile.pets) {
      if (profile.pets.pet_ownership !== undefined)
        dbFormat.pet_ownership = Array.isArray(profile.pets.pet_ownership)
          ? profile.pets.pet_ownership
          : [];
      if (profile.pets.pet_tolerance !== undefined)
        dbFormat.pet_tolerance = Array.isArray(profile.pets.pet_tolerance)
          ? profile.pets.pet_tolerance
          : [];
      if (profile.pets.pet_allergies !== undefined)
        dbFormat.pet_allergies = Array.isArray(profile.pets.pet_allergies)
          ? profile.pets.pet_allergies
          : [];
    }

    // Amenities
    if (profile.amenities) {
      if (profile.amenities.bathroom_pref !== undefined)
        dbFormat.bathroom_pref = profile.amenities.bathroom_pref;
      if (profile.amenities.laundry !== undefined)
        dbFormat.laundry = profile.amenities.laundry;
      if (profile.amenities.parking !== undefined)
        dbFormat.parking = profile.amenities.parking;
      if (profile.amenities.furnishing !== undefined)
        dbFormat.furnishing = profile.amenities.furnishing;
      if (profile.amenities.extras !== undefined)
        dbFormat.extras = Array.isArray(profile.amenities.extras)
          ? profile.amenities.extras
          : [];
    }

    return dbFormat;
  }

  // Add this method to your authService.js
  async saveQuizAnswer(field, value) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      const { error } = await supabase
        .from("users")
        .update({ [field]: value })
        .eq("id", user.id);

      if (error) throw error;
      return { success: true };
    } catch (error) {
      console.error("Save Quiz Answer Error:", error);
      return {
        success: false,
        error: error.message || "Failed to save quiz answer",
      };
    }
  }

  async getQuizAnswers() {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      const fields = [
        "weekend_vibe",
        "sleep_schedule",
        "dish_washing",
        "friends_over",
        "pet_situation",
        "move_in_selection",
        "lease_duration",
        "budget_min",
        "budget_max",
        "roommates",
        "zone_drawn",
      ];

      const { data, error } = await supabase
        .from("users")
        .select(fields.join(","))
        .eq("id", user.id)
        .single();

      if (error) throw error;
      return data;
    } catch (err) {
      console.error("getQuizAnswers Error:", err);
      return {};
    }
  }

  // Add this to your authService
  async removeUserFlag(flagId) {
    const { error } = await supabase
      .from("user_flags")
      .delete()
      .eq("id", flagId);

    if (error) {
      throw error;
    }
  }

  // authService.js
  async flagUser(flaggedUserId, flagType, reason = "") {
    const {
      data: { user },
      error,
    } = await supabase.auth.getUser();
    if (error || !user) throw error || new Error("No user");
    const { error: insertError } = await supabase.from("user_flags").insert([
      {
        flagged_user_id: flaggedUserId,
        flagger_user_id: user.id,
        flag_type: flagType,
        reason,
      },
    ]);
    if (insertError) throw insertError;
    return true;
  }

  async getUserFlags() {
    try {
      // Get the current authenticated user
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      // First, get all user flags for the current user
      const { data: flagsData, error: flagsError } = await supabase
        .from("user_flags")
        .select("id, flagged_user_id, flag_type, reason, created_at")
        .eq("flagger_user_id", user.id); // Use the current user's ID

      if (flagsError) {
        console.error("Supabase flags error:", flagsError);
        return [];
      }

      if (!flagsData || flagsData.length === 0) {
        // console.log("No flags found for user");
        return [];
      }

      // Get unique user IDs from the flags
      const userIds = [
        ...new Set(flagsData.map((flag) => flag.flagged_user_id)),
      ];

      // Get user information for all flagged users
      const { data: usersData, error: usersError } = await supabase
        .from("users")
        .select("id, first_name, last_name, email")
        .in("id", userIds);

      if (usersError) {
        console.error("Supabase users error:", usersError);
        // Continue with flags but without user data
      }

      // Create a map of user ID to user data for quick lookup
      const usersMap = {};
      if (usersData) {
        usersData.forEach((user) => {
          usersMap[user.id] = user;
        });
      }

      // // Debug: Print all results for inspection
      // // console.log("Raw flag records from Supabase:", flagsData);
      // // console.log("User data from Supabase:", usersData);

      // Combine flags with user data
      const processed = flagsData.map((flag) => {
        const user = usersMap[flag.flagged_user_id];
        return {
          ...flag,
          flagged_user: user
            ? {
                first_name: user.first_name || "(No first name)",
                last_name: user.last_name || "(No last name)",
                email: user.email || "(No email)",
              }
            : {
                first_name: "(Unknown)",
                last_name: "User",
                email: "(user deleted)",
              },
        };
      });

      // Debug: Highlight any flags where the user doesn't exist
      // const missingUsers = processed.filter(item => !usersMap[item.flagged_user_id]);
      // if (missingUsers.length > 0) {
      //   console.warn("Warning: These flags reference users missing from the users table:",
      //     missingUsers.map(i => i.flagged_user_id));
      // }

      // Debug: Show processed flags as displayed in the app
      // processed.forEach(flag => {
      //   const user = flag.flagged_user;
      //   // console.log(
      //     `[${flag.flag_type}]`,
      //     flag.flagged_user_id,
      //     `${user.first_name} ${user.last_name}`,
      //     user.email,
      //     `Reason: ${flag.reason || "(none)"}`
      //   );
      // });

      return processed;
    } catch (err) {
      console.error("getUserFlags Error:", err);
      return [];
    }
  }

  async setProfileVisibility(isVisible) {
    const {
      data: { user },
      error,
    } = await supabase.auth.getUser();
    if (error || !user) throw error || new Error("No user");
    const { error: updateError } = await supabase
      .from("users")
      .update({ is_visible: isVisible })
      .eq("id", user.id);
    if (updateError) throw updateError;
    return true;
  }

  async getUserIntent() {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      const { data, error } = await supabase
        .from("users")
        .select("intent")
        .eq("id", user.id)
        .single();

      if (error || !data)
        throw error || new Error("Failed to fetch user intent");
      return data.intent;
    } catch (err) {
      console.error("getUserIntent Error:", err);
      return null;
    }
  }

  async setOnboardingStep(step) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      const { data, error } = await supabase
        .from("users")
        .update({ onboarding: step })
        .eq("id", user.id) // <-- fix: use user.id, not user
        .select("onboarding")
        .single();

      if (error || !data)
        throw error || new Error("Failed to update onboarding step");
      return data.onboarding;
    } catch (err) {
      console.error("setOnboardingStep Error:", err);
      return null;
    }
  }

  async getOnboardingStep(userId) {
    try {
      const { data, error } = await supabase
        .from("users")
        .select("onboarding")
        .eq("id", userId)
        .maybeSingle(); // ← no error when 0 rows

      if (error) throw error;
      return data?.onboarding ?? null; // caller can decide default
    } catch (err) {
      console.error("getOnboardingStep Error:", err);
      return null;
    }
  }

  async updatePassword(newPassword) {
    // Only for email/password users!
    const { error } = await supabase.auth.updateUser({ password: newPassword });
    if (error) throw error;
    return { success: true };
  }

  async deleteAccount() {
    const {
      data: { user },
      error: userError,
    } = await supabase.auth.getUser();
    if (userError) throw userError;
    if (!user) throw new Error("No authenticated user");

    // 1. Get user data
    const { data: userData, error: fetchError } = await supabase
      .from("users")
      .select("*")
      .eq("id", user.id)
      .single();
    if (fetchError) throw fetchError;

    // 2. Insert into deleted_users
    const { error: insertError } = await supabase
      .from("deleted_users")
      .insert([{ ...userData, deleted_at: new Date().toISOString() }]);
    if (insertError) throw insertError;

    // 3. Remove user from users table
    const { error: deleteError } = await supabase
      .from("users")
      .delete()
      .eq("id", user.id);
    if (deleteError) throw deleteError;

    // 4. Optionally sign out the user
    await supabase.auth.signOut();

    return { success: true };
  }

  async getCurrentSession() {
    try {
      const {
        data: { session },
        error,
      } = await supabase.auth.getSession();
      if (error) throw error;
      return { success: true, session };
    } catch (error) {
      console.error("Get Current Session Error:", error);
      return {
        success: false,
        error: error.message || "Failed to get current session",
      };
    }
  }

  async updateUserProfile(userId, updates) {
    try {
      const { data, error } = await supabase
        .from("profiles")
        .upsert({
          id: userId,
          ...updates,
          updated_at: new Date().toISOString(),
        })
        .select();
      if (error) throw error;
      return { success: true, data };
    } catch (error) {
      console.error("Update Profile Error:", error);
      return {
        success: false,
        error: error.message || "Failed to update profile",
      };
    }
  }

  async resetPassword(email) {
    try {
      const { error } = await supabase.auth.resetPasswordForEmail(email, {
        redirectTo: this.redirectUrl,
      });
      if (error) throw error;
      return { success: true };
    } catch (error) {
      console.error("Reset Password Error:", error);
      return {
        success: false,
        error: error.message || "Failed to send reset email",
      };
    }
  }

  onAuthStateChange(callback) {
    return supabase.auth.onAuthStateChange((event, session) => {
      callback(event, session);
    });
  }

  async checkEmailVerification() {
    try {
      const {
        data: { user },
      } = await supabase.auth.getUser();
      if (user) {
        return {
          success: true,
          isVerified: user.email_confirmed_at !== null,
          user,
        };
      }
      return { success: false, error: "No user found" };
    } catch (error) {
      console.error("Check Email Verification Error:", error);
      return {
        success: false,
        error: error.message || "Failed to check email verification",
      };
    }
  }

  // async saveQuizAnswer(fieldOrObject, value) {
  //   try {
  //     const { data: { user }, error: userError } = await supabase.auth.getUser();
  //     if (userError) throw userError;
  //     if (!user) throw new Error("No authenticated user");

  //     const updatePayload =
  //       typeof fieldOrObject === "object" ? fieldOrObject : { [fieldOrObject]: value };

  //     const { error } = await supabase
  //       .from('users')
  //       .update(updatePayload)
  //       .eq('id', user.id);

  //     if (error) throw error;
  //     return { success: true };
  //   } catch (error) {
  //     console.error('Save Quiz Answer Error:', error);
  //     return { success: false, error: error.message || 'Failed to save quiz answer' };
  //   }
  // }

  async fetchCurrentUser() {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      // Fetch user data from your users table
      const { data, error } = await supabase
        .from("users")
        .select(
          "first_name, last_name, email, location_sharing, verification_level"
        )
        .eq("id", user.id)
        .single();

      if (error) throw error;

      return {
        success: true,
        user: user, // Auth user object
        userData: data, // Custom user data from users table
      };
    } catch (err) {
      console.error("fetchCurrentUser Error:", err);
      return {
        success: false,
        error: err.message || "Failed to fetch user data",
      };
    }
  }

  async setLocationSharing(isSharing) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      const { error } = await supabase
        .from("users")
        .update({ location_sharing: isSharing })
        .eq("id", user.id);

      if (error) throw error;
      return { success: true };
    } catch (error) {
      console.error("setLocationSharing Error:", error);
      return {
        success: false,
        error: error.message || "Failed to save location preference",
      };
    }
  }

  async setUserVerificationBronze() {
    const {
      data: { user },
      error,
    } = await supabase.auth.getUser();
    if (error || !user) throw error || new Error("No user");

    // Only update if currently null or empty or not set to bronze, silver, gold, platinum
    const { data: userData, error: fetchError } = await supabase
      .from("users")
      .select("verification_level")
      .eq("id", user.id)
      .single();

    if (fetchError) throw fetchError;

    // If the level is already set, do not overwrite!
    if (
      userData &&
      userData.verification_level &&
      ["bronze", "silver", "gold", "platinum"].includes(
        userData.verification_level
      )
    ) {
      return false; // Already set
    }

    // Otherwise, set to bronze
    const { error: updateError } = await supabase
      .from("users")
      .update({ verification_level: "bronze" })
      .eq("id", user.id);

    if (updateError) throw updateError;
    return true;
  }

  // Updated updateUserProfile method
  async updateUserProfile(updates) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      const { data, error } = await supabase
        .from("users")
        .update({
          ...updates,
          updated_at: new Date().toISOString(),
        })
        .eq("id", user.id)
        .select()
        .single();

      if (error) throw error;
      return { success: true, data };
    } catch (error) {
      console.error("Update Profile Error:", error);
      return {
        success: false,
        error: error.message || "Failed to update profile",
      };
    }
  }

  async resendEmailVerification(email) {
    try {
      const { error } = await supabase.auth.resend({ type: "signup", email });
      if (error) throw error;
      return { success: true };
    } catch (error) {
      console.error("Resend Email Verification Error:", error);
      return {
        success: false,
        error: error.message || "Failed to resend verification email",
      };
    }
  }

  // KEEP ONLY ONE of this function in the class:
  async getAccountProfileForUI() {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");

      const { data, error } = await supabase
        .from("users")
        .select(
          `
          id,
          first_name, last_name, email,
          about, interests, photos,
          education, company,
          weekend_vibe, sleep_schedule, dish_washing, friends_over, pet_situation,
          budget_min, budget_max,
          move_in_selection, lease_duration, zone_drawn,
          zipcode, location_sharing
        `
        )

        .eq("id", user.id)
        .single();

      if (error) throw error;

      const profile = dbUserToUiProfile(data || {});
      return { success: true, profile, raw: data };
    } catch (err) {
      console.error("getAccountProfileForUI Error:", err);
      return {
        success: false,
        error: err.message || "Failed to load account profile",
      };
    }
  }

  async updateAccountProfileFromUI(uiProfile) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");
      // console.log("incoming ui", uiProfile);

      const updates = {};

      // Handle names
      if (uiProfile.name) {
        const parts = uiProfile.name.trim().split(/\s+/);
        updates.first_name = parts[0] || null;
        updates.last_name = parts.length > 1 ? parts.slice(1).join(" ") : null;
      }

      // Build updates from helper
      const dbUpdates = uiProfileToDbUpdates(uiProfile);
      Object.entries(dbUpdates).forEach(([key, value]) => {
        if (value !== undefined) updates[key] = value;
      });

      updates.updated_at = new Date().toISOString();

      // Step 1: Perform update (don’t care about returned cols here)
      const { error: updateError } = await supabase
        .from("users")
        .update(updates)
        .eq("id", user.id);

      if (updateError) throw updateError;

      // Step 2: Fetch the full record after update
      // Step 2: Fetch the full record after update
      // Step 2: Fetch the full record after update
      const { data: fullData, error: fetchError } = await supabase
        .from("users")
        .select(
          `
  id,
  first_name, last_name, email,
  about, interests, photos,
  education, company,
  weekend_vibe, sleep_schedule, dish_washing, friends_over, pet_situation,
  budget_min, budget_max,
  move_in_selection, lease_duration, zone_drawn, location_sharing, zipcode
`
        )
        .eq("id", user.id)
        .single();

      if (fetchError) throw fetchError;

      return {
        success: true,
        profile: dbUserToUiProfile(fullData),
        raw: fullData,
      };
    } catch (err) {
      console.error("updateAccountProfileFromUI Error:", err);
      return {
        success: false,
        error: err.message || "Failed to update account profile",
      };
    }
  }

  // 👇 small helper to get current user
  async _getCurrentUserOrThrow() {
    const { data, error } = await supabase.auth.getUser();
    if (error) throw error;
    const user = data?.user;
    if (!user) throw new Error("No authenticated user");
    return user;
  }

  async uploadUserPhoto(
    fileUri,
    {
      bucket = "user-photos",
      contentType = "image/jpeg",
    } = {}
  ) {
    try {
      const user = await this._getCurrentUserOrThrow();
  
      const fileExt = fileUri.split(".").pop() || "jpg";
      const fileName = `${Date.now()}-${Math.random()
        .toString(36)
        .substring(2)}.${fileExt}`;
      const filePath = `${user.id}/${fileName}`;
  
      // Upload file
      const response = await fetch(fileUri);
      if (!response.ok) throw new Error(`Failed to fetch file: ${response.status}`);
      const fileData = await response.arrayBuffer();
  
      const { error: uploadError } = await supabase.storage
        .from(bucket)
        .upload(filePath, fileData, {
          contentType,
          upsert: false,
        });
  
      if (uploadError) throw uploadError;
  
      // Generate public URL (no need for signed URLs anymore)
      const { data: urlData } = supabase.storage
        .from(bucket)
        .getPublicUrl(filePath);
  
      const publicUrl = urlData.publicUrl;
  
      // Update user row with file path
      const { data: userRow, error: readErr } = await supabase
        .from("users")
        .select("photos")
        .eq("id", user.id)
        .single();
  
      if (readErr) throw readErr;
  
      const currentPhotos = Array.isArray(userRow?.photos) ? userRow.photos : [];
      const updatedPhotos = [...currentPhotos, filePath];
  
      const { error: updateErr } = await supabase
        .from("users")
        .update({
          photos: updatedPhotos,
          updated_at: new Date().toISOString(),
        })
        .eq("id", user.id);
  
      if (updateErr) throw updateErr;
  
      return {
        success: true,
        path: filePath,
        url: publicUrl, // Never expires!
        photos: updatedPhotos,
      };
    } catch (error) {
      console.error("❌ [uploadUserPhoto] ERROR:", error.message);
      return {
        success: false,
        error: error.message,
        data: null,
        url: null,
      };
    }
  }

  async updateZip(newZip) {
    try {
      const {
        data: { user },
        error: userError,
      } = await supabase.auth.getUser();
      if (userError) throw userError;
      if (!user) throw new Error("No authenticated user");
  
      // Ensure we only allow numeric zips (basic validation)
      const sanitizedZip = String(newZip).replace(/\D/g, "").slice(0, 10);
  
      if (!sanitizedZip) {
        throw new Error("Invalid zipcode provided");
      }
  
      const { data, error } = await supabase
        .from("users")
        .update({
          zipcode: sanitizedZip,
          updated_at: new Date().toISOString(),
        })
        .eq("id", user.id)
        .select("id, zipcode")
        .single();
  
      if (error) throw error;
  
      return {
        success: true,
        zipcode: data.zipcode,
      };
    } catch (err) {
      console.error("updateZip Error:", err);
      return {
        success: false,
        error: err.message || "Failed to update zipcode",
      };
    }
  }
  
  
  // Simplified helper - no need for signed URL logic
  async handleUploadSuccess(data, filePath, bucket) {
    try {
      // Always generate public URL since bucket is public
      const { data: urlData } = supabase.storage
        .from(bucket)
        .getPublicUrl(filePath);
      
      const publicUrl = urlData.publicUrl;
      
      return {
        success: true,
        error: null,
        data: data,
        url: publicUrl,
        path: filePath,
      };
    } catch (urlError) {
      console.error("❌ [handleUploadSuccess] URL generation error:", urlError);
      return {
        success: true, // Upload still worked
        error: null,
        data: data,
        url: null,
        path: filePath,
      };
    }
  }
  
  async deleteUserPhoto(photoPath, { bucket = "user-photos" } = {}) {
    try {
      const user = await this._getCurrentUserOrThrow();
  
      // 1) Get current photos array
      const { data: userRow, error: readErr } = await supabase
        .from("users")
        .select("photos")
        .eq("id", user.id)
        .single();
  
      if (readErr) throw readErr;
  
      const currentPhotos = Array.isArray(userRow?.photos) ? userRow.photos : [];
      
      // 2) Remove from storage
      const { error: deleteError } = await supabase.storage
        .from(bucket)
        .remove([photoPath]);
  
      // Don't throw on storage delete errors - file might already be gone
      if (deleteError) {
        console.warn("⚠️ Storage deletion warning:", deleteError.message);
      }
  
      // 3) Always update database (even if storage delete failed)
      const updatedPhotos = currentPhotos.filter(path => path !== photoPath);
      
      const { error: updateErr } = await supabase
        .from("users")
        .update({
          photos: updatedPhotos,
          updated_at: new Date().toISOString(),
        })
        .eq("id", user.id);
  
      if (updateErr) throw updateErr;
  
      return { 
        success: true, 
        photos: updatedPhotos,
        storageDeleted: !deleteError
      };
    } catch (err) {
      console.error("❌ deleteUserPhoto error:", err);
      return {
        success: false,
        error: err.message || "Delete failed",
      };
    }
  }
  
  // Simplified getUserPhotos - no expiration concerns!
  async getUserPhotos({ bucket = "user-photos" } = {}) {
    try {
      const user = await this._getCurrentUserOrThrow();
  
      // 1) Load paths from users.photos
      const { data, error } = await supabase
        .from("users")
        .select("photos")
        .eq("id", user.id)
        .single();
  
      if (error) throw error;
  
      const paths = Array.isArray(data?.photos) ? data.photos : [];
      
      if (paths.length === 0) {
        return { success: true, photos: [], paths: [] };
      }
  
      // 2) Generate public URLs (never expire)
      const urls = paths.map(path => 
        supabase.storage.from(bucket).getPublicUrl(path).data.publicUrl
      );
  
      return { 
        success: true, 
        photos: urls, 
        paths 
      };
    } catch (err) {
      console.error("❌ getUserPhotos error:", err);
      return {
        success: false,
        error: err.message || "Fetch failed",
        photos: [],
        paths: []
      };
    }
  }
}

// === helpers (fix bug + align to your schema) ===
const toInitials = (fullName) =>
  (fullName || "User")
    .split(/\s+/)
    .filter(Boolean)
    .map((s) => s[0] || "")
    .join("")
    .slice(0, 2)
    .toUpperCase() || "UU";

const formatBudgetRange = (min, max) => {
  if (min && max) return `$${min}–$${max}`;
  if (min && !max) return `$${min}+`;
  if (!min && max) return `Up to $${max}`;
  return "";
};

const parseBudgetRange = (label) => {
  if (!label) return { min: null, max: null };
  const clean = String(label).replace(/[\$,]/g, "");
  const mRange = clean.match(/(\d+)\s*[–-]\s*(\d+)/);
  if (mRange)
    return { min: parseInt(mRange[1], 10), max: parseInt(mRange[2], 10) };
  const mUpTo = clean.match(/up to\s*(\d+)/i);
  if (mUpTo) return { min: null, max: parseInt(mUpTo[1], 10) };
  const mPlus = clean.match(/(\d+)\s*\+/);
  if (mPlus) return { min: parseInt(mPlus[1], 10), max: null };
  const lone = clean.match(/^\d+$/);
  if (lone) return { min: parseInt(clean, 10), max: null };
  return { min: null, max: null };
};

const normalizePhotos = (raw) => {
  if (!Array.isArray(raw)) return [];
  return raw
    .map((p, i) => {
      if (typeof p === "string") return { id: String(i), uri: p };
      if (p && typeof p === "object" && p.uri)
        return { id: p.id || String(i), uri: p.uri };
      return null;
    })
    .filter(Boolean);
};

function computeProfileStrength(row) {
  const checks = [
    !!row?.about,
    Array.isArray(row?.interests) && row.interests.length >= 3,
    !!row?.budget_min || !!row?.budget_max,
    !!row?.move_in_selection,
    !!row?.lease_duration,
    !!row?.education || !!row?.company,
    Array.isArray(row?.photos) && row.photos.length > 0,
  ];
  const pct = Math.round((checks.filter(Boolean).length / checks.length) * 100);
  return Number.isFinite(pct) ? pct : 0;
}

// weekend_vibe is a string in your data (e.g., "outdoor")
// Use a simple heuristic to decide "drinks"
const drinksFromWeekendVibe = (v) => {
  if (!v) return false;
  const s = String(v).toLowerCase();
  return ["party", "bar", "drinks", "nightlife"].some((k) => s.includes(k));
};



// pet_situation: "dog", "small_pet", "none", ...
function dbUserToUiProfile(row) {
  // console.log("row from database", row);
  const fullName =
    [row?.first_name?.trim(), row?.last_name?.trim()]
      .filter(Boolean)
      .join(" ") || null;

  const drinks = drinksFromWeekendVibe(row?.weekend_vibe);
  const dogOwner = row?.pet_situation === "dog";
  const petFriendly = !!row?.pet_situation && row.pet_situation !== "none";
  const nonSmoker = true; // default until tracked explicitly

  // ✅ define zipcode once
  const zipcode = row?.zipcode || null;

  return {
    avatarUri: null,
    initials: toInitials(fullName),
    name: fullName || "New User",
    location_sharing: row.location_sharing,

    // ✅ now safe to use
    location: zipcode ? `ZIP ${zipcode}` : null,
    rawZipcode: zipcode,

    about: row?.about || "",
    interests: Array.isArray(row?.interests)
      ? row.interests.filter((x) => typeof x === "string")
      : [],

    lifestyle: { drinks, dogOwner, nonSmoker, petFriendly },

    housing: {
      budget: formatBudgetRange(row?.budget_min, row?.budget_max),
      moveIn: row?.move_in_selection || "",
      lease: row?.lease_duration || "",
      roomSize: row?.zone_drawn || "",
    },

    basic: {
      education: row?.education || "",
      company: row?.company || "",
    },

    photos: normalizePhotos(row?.photos),
    strength: computeProfileStrength(row),
  };
}

function uiProfileToDbUpdates(ui) {
  const updates = {};
  // console.log("backend ", ui);

  if ("about" in ui) updates.about = ui.about ?? null;
  if ("interests" in ui)
    updates.interests = Array.isArray(ui.interests) ? ui.interests : undefined;
  if ("photos" in ui) {
    updates.photos = Array.isArray(ui.photos)
      ? ui.photos
          .map((p) => (typeof p === "string" ? p : p?.uri))
          .filter(Boolean)
      : undefined;
  }

  if (ui.lifestyle) {
    if ("drinks" in ui.lifestyle)
      updates.weekend_vibe = ui.lifestyle.drinks ? "party" : null;
    if ("dogOwner" in ui.lifestyle || "petFriendly" in ui.lifestyle) {
      updates.pet_situation = ui.lifestyle.dogOwner
        ? "dog"
        : ui.lifestyle.petFriendly
        ? "small_pet"
        : "none";
    }
  }

  if (ui.housing) {
    if ("budget" in ui.housing) {
      const { min, max } = parseBudgetRange(ui.housing.budget);
      updates.budget_min = min;
      updates.budget_max = max;
    }
    if ("moveIn" in ui.housing) updates.move_in_selection = ui.housing.moveIn;
    if ("lease" in ui.housing) updates.lease_duration = ui.housing.lease;
    if ("roomSize" in ui.housing) updates.zone_drawn = ui.housing.roomSize;
  }

  if (ui.basic) {
    if ("education" in ui.basic) updates.education = ui.basic.education ?? null;
    if ("company" in ui.basic) updates.company = ui.basic.company ?? null;
  }
  if ("location_sharing" in ui) {
    updates.location_sharing = ui.location_sharing;
  }

  // ✅ Persist location_sharing
  if ("location" in ui) {
    updates.zipcode = ui.location?.replace("ZIP ", "") || null;
  }

  if ("location_sharing" in ui) {
    updates.location_sharing = ui.location_sharing;
  }

  return updates;
}

export const authService = new AuthService();
export default authService;

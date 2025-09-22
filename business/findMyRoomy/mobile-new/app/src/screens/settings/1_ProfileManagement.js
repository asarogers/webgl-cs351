import {
  Ionicons,
  MaterialCommunityIcons,
  MaterialIcons,
} from "@expo/vector-icons";
import React, { useEffect, useState } from "react";
import {
  ActivityIndicator,
  Alert,
  Animated,
  KeyboardAvoidingView,
  Platform,
  ScrollView,
  StatusBar,
  StyleSheet,
  Switch,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import UpgradeVerificationModal from "../../../../components/UpgradeVerificationModal"; // adjust path
import authService from "../../../../database/authService"; // update path as needed
import SettingsHeader from "./SettingsHeader";

const AccountManagement = ({ setPage }) => {
  const [formData, setFormData] = useState({
    firstName: "",
    lastName: "",
    email: "",
    currentPassword: "",
    newPassword: "",
    confirmPassword: "",
  });
  const [locationSharing, setLocationSharing] = useState(true);
  const [verificationLevel, setVerificationLevel] = useState("bronze");
  const [hasChanges, setHasChanges] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [showPasswords, setShowPasswords] = useState({
    current: false,
    new: false,
    confirm: false,
  });
  const [errors, setErrors] = useState({});
  const [fadeAnim] = useState(new Animated.Value(0));
  const [userHasPassword, setUserHasPassword] = useState(true);
  const [showUpgradeModal, setShowUpgradeModal] = useState(false);

  // Fetch user data on mount
  useEffect(() => {
    Animated.timing(fadeAnim, {
      toValue: 1,
      duration: 300,
      useNativeDriver: true,
    }).start();

    const fetchUser = async () => {
      setIsLoading(true);
      try {
        const { success, user, userData, error } =
          await authService.fetchCurrentUser();

        if (!success) {
          console.error("Failed to fetch user:", error);
          Alert.alert("Error", "Failed to load user data");
          return;
        }

        // Detect social auth (e.g., Google/Apple)
        const provider =
          user?.app_metadata?.provider || user?.identities?.[0]?.provider;
        setUserHasPassword(provider === "email");

        // Set form data from userData
        if (userData) {
          setFormData((prev) => ({
            ...prev,
            firstName: userData.first_name || "",
            lastName: userData.last_name || "",
            email: userData.email || user.email || "",
          }));
          setLocationSharing(userData.location_sharing !== false);
          setVerificationLevel(userData.verification_level || "bronze");
        }
      } catch (e) {
        console.error("Fetch user error:", e);
        Alert.alert("Error", "Failed to load user data");
      } finally {
        setIsLoading(false);
      }
    };

    fetchUser();
  }, []);

  // Validators
  const validateEmail = (email) => /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  const validatePassword = (password) => {
    if (password.length < 8) return "Password must be at least 8 characters";
    if (!/(?=.*[a-z])/.test(password))
      return "Password must contain lowercase letter";
    if (!/(?=.*[A-Z])/.test(password))
      return "Password must contain uppercase letter";
    if (!/(?=.*\d)/.test(password)) return "Password must contain a number";
    return null;
  };

  // Handlers
  const handleInputChange = (field, value) => {
    setFormData((prev) => ({ ...prev, [field]: value }));
    setHasChanges(true);

    if (errors[field]) setErrors((prev) => ({ ...prev, [field]: null }));

    if (field === "email" && value && !validateEmail(value)) {
      setErrors((prev) => ({
        ...prev,
        email: "Please enter a valid email address",
      }));
    }
    if (field === "newPassword" && value) {
      const passwordError = validatePassword(value);
      setErrors((prev) => ({ ...prev, newPassword: passwordError }));
    }
    if (
      field === "confirmPassword" &&
      value &&
      formData.newPassword !== value
    ) {
      setErrors((prev) => ({
        ...prev,
        confirmPassword: "Passwords don't match",
      }));
    }
  };

  const handleLocationToggle = async (value) => {
    setLocationSharing(value);
    setHasChanges(true);
    try {
      // Use setLocationSharing instead of updateUserProfile
      const result = await authService.setLocationSharing(value);
      if (!result.success) {
        throw new Error(result.error);
      }
    } catch (e) {
      Alert.alert("Error", "Failed to update location sharing.");
      setLocationSharing(!value); // Revert on error
    }
  };

  const togglePasswordVisibility = (field) => {
    setShowPasswords((prev) => ({ ...prev, [field]: !prev[field] }));
  };

  const handleSaveChanges = async () => {
    // Validation
    const newErrors = {};
    if (!formData.firstName.trim())
      newErrors.firstName = "First name is required";
    if (!formData.lastName.trim()) newErrors.lastName = "Last name is required";
    if (!validateEmail(formData.email))
      newErrors.email = "Valid email is required";

    if (formData.newPassword) {
      if (userHasPassword && !formData.currentPassword)
        newErrors.currentPassword =
          "Current password required to change password";
      const passwordError = validatePassword(formData.newPassword);
      if (passwordError) newErrors.newPassword = passwordError;
      if (formData.newPassword !== formData.confirmPassword)
        newErrors.confirmPassword = "Passwords don't match";
    }

    if (Object.keys(newErrors).length > 0) {
      setErrors(newErrors);
      Alert.alert("Validation Error", "Please fix the errors below");
      return;
    }

    setIsLoading(true);
    try {
      // Update profile fields
      const profileResult = await authService.updateUserProfile({
        first_name: formData.firstName,
        last_name: formData.lastName,
        email: formData.email,
      });

      if (!profileResult.success) {
        throw new Error(profileResult.error);
      }

      // Update password if needed (only for email/password users)
      if (formData.newPassword && userHasPassword) {
        const passwordResult = await authService.updatePassword(
          formData.newPassword
        );
        if (!passwordResult.success) {
          throw new Error(passwordResult.error);
        }
      }

      setHasChanges(false);
      setFormData((prev) => ({
        ...prev,
        currentPassword: "",
        newPassword: "",
        confirmPassword: "",
      }));
      Alert.alert("Success", "Your profile has been updated!");
    } catch (error) {
      Alert.alert("Error", error.message || "Failed to save changes.");
    } finally {
      setIsLoading(false);
    }
  };

  const handleDeleteAccount = () => {
    Alert.alert(
      "Delete Account",
      "This action will permanently delete your account and all associated data. This cannot be undone.\n\nAre you absolutely sure?",
      [
        { text: "Cancel", style: "cancel" },
        {
          text: "Delete Forever",
          style: "destructive",
          onPress: async () => {
            setIsLoading(true);
            try {
              const result = await authService.deleteAccount();
              if (result.success) {
                Alert.alert(
                  "Account Deleted",
                  "Your account has been permanently deleted."
                );
                setTimeout(() => setPage("login"), 1200);
              } else {
                Alert.alert(
                  "Error",
                  result.error || "Could not delete account"
                );
              }
            } catch (err) {
              Alert.alert("Error", err.message || "Could not delete account");
            } finally {
              setIsLoading(false);
            }
          },
        },
      ]
    );
  };

  // Verification logic (unchanged)
  const getVerificationDisplay = (level) => {
    const configs = {
      bronze: {
        icon: "medal-outline",
        text: "Bronze Verified",
        textColor: "#FFFFFF",
        style: {
          background: ["#B87333", "#8B4513"],
          borderColor: "#FFFFFF",
          shadowColor: "#B87333",
        },
      },
      silver: {
        icon: "star-outline",
        text: "Silver Verified",
        textColor: "#1F2937",
        style: {
          background: ["#C0C0C0", "#A9A9A9"],
          borderColor: "#FFFFFF",
          shadowColor: "#C0C0C0",
        },
      },
      gold: {
        icon: "trophy-outline",
        text: "Gold Verified",
        textColor: "#000000",
        style: {
          background: ["#FFD700", "#DAA520"],
          borderColor: "#FFFFFF",
          shadowColor: "#FFD700",
        },
      },
      platinum: {
        icon: "diamond-outline",
        text: "Platinum Verified",
        textColor: "#1F2937",
        style: {
          background: ["#E5E4E2", "#D3D3D3"],
          borderColor: "#FFFFFF",
          shadowColor: "#E5E4E2",
        },
      },
      unverified: {
        icon: "alert-circle-outline",
        text: "Not Verified",
        textColor: "#FFFFFF",
        style: {
          background: ["#9CA3AF", "#6B7280"],
          borderColor: "#FFFFFF",
          shadowColor: "#9CA3AF",
        },
      },
    };
    return configs[level] || configs.unverified;
  };

  const verification = getVerificationDisplay(verificationLevel);

  const renderTextInput = (field, label, props = {}) => (
    <View style={styles.inputGroup}>
      <Text style={styles.inputLabel}>{label}</Text>
      <View style={styles.inputContainer}>
        <TextInput
          style={[
            styles.textInput,
            errors[field] && styles.textInputError,
            props.secureTextEntry && styles.passwordInput,
          ]}
          value={formData[field]}
          onChangeText={(value) => handleInputChange(field, value)}
          placeholderTextColor="#9CA3AF"
          {...props}
        />
        {props.secureTextEntry && (
          <TouchableOpacity
            style={styles.eyeIcon}
            onPress={() => togglePasswordVisibility(field)}
          >
            <Ionicons
              name={showPasswords[field] ? "eye-off" : "eye"}
              size={20}
              color="#9CA3AF"
            />
          </TouchableOpacity>
        )}
      </View>
      {errors[field] && <Text style={styles.errorText}>{errors[field]}</Text>}
    </View>
  );

  return (
    <SafeAreaView style={styles.container}>
      <StatusBar barStyle="light-content" backgroundColor="#1F2937" />
      <SettingsHeader
        title="Account Settings"
        subtitle="Manage your account information"
        onBack={() => setPage("home")}
      />

      <KeyboardAvoidingView
        style={styles.keyboardContainer}
        behavior={Platform.OS === "ios" ? "padding" : "height"}
      >
        <ScrollView
          style={styles.scrollContainer}
          showsVerticalScrollIndicator={false}
          keyboardShouldPersistTaps="handled"
        >
          <Animated.View style={{ opacity: fadeAnim }}>
            {/* Show loading indicator while fetching data */}
            {isLoading && !formData.firstName && (
              <View style={styles.loadingContainer}>
                <ActivityIndicator size="large" color="#3B82F6" />
                <Text style={styles.loadingText}>Loading your profile...</Text>
              </View>
            )}

            {/* Verification card */}
            <View style={styles.verificationSection}>
              <View style={styles.verificationCard}>
                <View
                  style={[
                    styles.verificationGradient,
                    {
                      backgroundColor: verification.style.background[0],
                      borderColor: verification.style.borderColor,
                      shadowColor: verification.style.shadowColor,
                    },
                  ]}
                >
                  <View style={styles.verificationContent}>
                    <View style={styles.verificationIconContainer}>
                      <Ionicons
                        name={verification.icon}
                        size={24}
                        color={verification.textColor}
                      />
                    </View>
                    <View style={styles.verificationTextContainer}>
                      <Text
                        style={[
                          styles.verificationTitle,
                          { color: verification.textColor },
                        ]}
                      >
                        {verification.text}
                      </Text>
                      <Text
                        style={[
                          styles.verificationSubtitle,
                          { color: verification.textColor, opacity: 0.8 },
                        ]}
                      >
                        {verificationLevel === "gold"
                          ? "Financial verification complete"
                          : verificationLevel === "silver"
                          ? "Identity verification complete"
                          : verificationLevel === "bronze"
                          ? "Basic verification complete"
                          : verificationLevel === "platinum"
                          ? "Full verification complete"
                          : "Complete verification to build trust"}
                      </Text>
                    </View>
                    <View style={styles.verificationLevelIndicator}>
                      <Text
                        style={[
                          styles.verificationLevel,
                          { color: verification.textColor },
                        ]}
                      >
                        {verificationLevel.toUpperCase()}
                      </Text>
                    </View>
                  </View>
                  <View style={styles.progressBarContainer}>
                    <View style={styles.progressBar}>
                      <View
                        style={[
                          styles.progressFill,
                          {
                            width:
                              verificationLevel === "bronze"
                                ? "25%"
                                : verificationLevel === "silver"
                                ? "50%"
                                : verificationLevel === "gold"
                                ? "75%"
                                : verificationLevel === "platinum"
                                ? "100%"
                                : "0%",
                            backgroundColor: verification.textColor,
                          },
                        ]}
                      />
                    </View>
                  </View>
                  {verificationLevel !== "platinum" && (
                    <TouchableOpacity
                      style={[
                        styles.upgradeButton,
                        { backgroundColor: `${verification.textColor}20` },
                      ]}
                      onPress={() => setShowUpgradeModal(true)}
                    >
                      <Text
                        style={[
                          styles.upgradeButtonText,
                          { color: verification.textColor },
                        ]}
                      >
                        Upgrade Verification
                      </Text>
                      <Ionicons
                        name="arrow-forward"
                        size={16}
                        color={verification.textColor}
                      />
                    </TouchableOpacity>
                  )}
                </View>
              </View>
            </View>

            <UpgradeVerificationModal
              visible={showUpgradeModal}
              onClose={() => setShowUpgradeModal(false)}
              // verificationLevel = {verificationLevel}
              verificationLevel={"gold"}
              onSelectLevel={(level) => {
                // Here you could call your API to request upgrade
                setShowUpgradeModal(false);
              }}
            />

            {/* Personal Information */}
            <View style={styles.section}>
              <View style={styles.sectionHeader}>
                <View
                  style={[
                    styles.sectionTitleDot,
                    { backgroundColor: "#3B82F6" },
                  ]}
                />
                <Text style={[styles.sectionTitle, { color: "#3B82F6" }]}>
                  PERSONAL INFORMATION
                </Text>
              </View>

              <View style={styles.formContainer}>
                <View style={styles.row}>
                  <View style={[styles.inputGroup, styles.halfWidth]}>
                    <Text style={styles.inputLabel}>First Name</Text>
                    <TextInput
                      style={[
                        styles.textInput,
                        errors.firstName && styles.textInputError,
                      ]}
                      value={formData.firstName}
                      onChangeText={(value) =>
                        handleInputChange("firstName", value)
                      }
                      placeholder="First Name"
                      placeholderTextColor="#9CA3AF"
                    />
                    {errors.firstName && (
                      <Text style={styles.errorText}>{errors.firstName}</Text>
                    )}
                  </View>
                  <View style={[styles.inputGroup, styles.halfWidth]}>
                    <Text style={styles.inputLabel}>Last Name</Text>
                    <TextInput
                      style={[
                        styles.textInput,
                        errors.lastName && styles.textInputError,
                      ]}
                      value={formData.lastName}
                      onChangeText={(value) =>
                        handleInputChange("lastName", value)
                      }
                      placeholder="Last Name"
                      placeholderTextColor="#9CA3AF"
                    />
                    {errors.lastName && (
                      <Text style={styles.errorText}>{errors.lastName}</Text>
                    )}
                  </View>
                </View>
                {renderTextInput("email", "Email Address", {
                  placeholder: "your.email@example.com",
                  keyboardType: "email-address",
                  autoCapitalize: "none",
                  autoComplete: "email",
                })}
              </View>
            </View>

            {/* Password Security - Only show for email/password users */}
            {userHasPassword && (
              <View style={styles.section}>
                <View style={styles.sectionHeader}>
                  <View
                    style={[
                      styles.sectionTitleDot,
                      { backgroundColor: "#EF4444" },
                    ]}
                  />
                  <Text style={[styles.sectionTitle, { color: "#EF4444" }]}>
                    PASSWORD & SECURITY
                  </Text>
                </View>

                <View style={styles.formContainer}>
                  {renderTextInput("currentPassword", "Current Password", {
                    placeholder: "Enter current password",
                    secureTextEntry: !showPasswords.current,
                    autoComplete: "current-password",
                  })}
                  {renderTextInput("newPassword", "New Password", {
                    placeholder: "Enter new password",
                    secureTextEntry: !showPasswords.new,
                    autoComplete: "new-password",
                  })}
                  {renderTextInput("confirmPassword", "Confirm New Password", {
                    placeholder: "Confirm new password",
                    secureTextEntry: !showPasswords.confirm,
                    autoComplete: "new-password",
                  })}
                  {formData.newPassword && (
                    <View style={styles.passwordStrength}>
                      <Text style={styles.passwordStrengthTitle}>
                        Password Requirements:
                      </Text>
                      <View style={styles.requirementsList}>
                        <Text
                          style={[
                            styles.requirement,
                            formData.newPassword.length >= 8
                              ? styles.requirementMet
                              : styles.requirementUnmet,
                          ]}
                        >
                          • At least 8 characters
                        </Text>
                        <Text
                          style={[
                            styles.requirement,
                            /(?=.*[a-z])/.test(formData.newPassword)
                              ? styles.requirementMet
                              : styles.requirementUnmet,
                          ]}
                        >
                          • One lowercase letter
                        </Text>
                        <Text
                          style={[
                            styles.requirement,
                            /(?=.*[A-Z])/.test(formData.newPassword)
                              ? styles.requirementMet
                              : styles.requirementUnmet,
                          ]}
                        >
                          • One uppercase letter
                        </Text>
                        <Text
                          style={[
                            styles.requirement,
                            /(?=.*\d)/.test(formData.newPassword)
                              ? styles.requirementMet
                              : styles.requirementUnmet,
                          ]}
                        >
                          • One number
                        </Text>
                      </View>
                    </View>
                  )}
                </View>
              </View>
            )}

            {/* Privacy Settings */}
            <View style={styles.section}>
              <View style={styles.sectionHeader}>
                <View
                  style={[
                    styles.sectionTitleDot,
                    { backgroundColor: "#8B5CF6" },
                  ]}
                />
                <Text style={[styles.sectionTitle, { color: "#8B5CF6" }]}>
                  PRIVACY SETTINGS
                </Text>
              </View>

              <View style={styles.settingsContainer}>
                <View style={styles.toggleCard}>
                  <View style={styles.toggleContent}>
                    <View
                      style={[
                        styles.iconCircle,
                        { backgroundColor: "#F3F4F6" },
                      ]}
                    >
                      <MaterialIcons
                        name="location-on"
                        size={22}
                        color="#8B5CF6"
                      />
                    </View>
                    <View style={styles.toggleText}>
                      <Text style={styles.toggleTitle}>Location Sharing</Text>
                      <Text style={styles.toggleSubtitle}>
                        {locationSharing
                          ? "Apps can access your location"
                          : "Location access disabled"}
                      </Text>
                    </View>
                    <Switch
                      value={locationSharing}
                      onValueChange={handleLocationToggle}
                      trackColor={{ false: "#E5E7EB", true: "#DBEAFE" }}
                      thumbColor={locationSharing ? "#3B82F6" : "#9CA3AF"}
                      ios_backgroundColor="#E5E7EB"
                    />
                  </View>
                </View>
                {/* Delete Account Button */}
                <View style={styles.deleteCard}>
                  <TouchableOpacity
                    style={styles.deleteButton}
                    onPress={handleDeleteAccount}
                    activeOpacity={0.7}
                  >
                    <MaterialCommunityIcons
                      name="delete-forever"
                      size={22}
                      color="#DC2626"
                      style={styles.buttonIcon}
                    />
                    <Text style={styles.deleteButtonText}>Delete Account</Text>
                  </TouchableOpacity>
                </View>
              </View>
            </View>
            {/* Bottom spacing for buttons */}
            <View style={styles.bottomSpacing} />
          </Animated.View>
        </ScrollView>

        {/* Action Buttons */}
        <View style={styles.buttonContainer}>
          <TouchableOpacity
            style={[
              styles.saveButton,
              (!hasChanges || isLoading) && styles.saveButtonDisabled,
            ]}
            onPress={handleSaveChanges}
            disabled={!hasChanges || isLoading}
            activeOpacity={0.8}
          >
            {isLoading ? (
              <ActivityIndicator color="#fff" size="small" />
            ) : (
              <Text style={styles.saveButtonText}>Update Profile</Text>
            )}
          </TouchableOpacity>
        </View>
      </KeyboardAvoidingView>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: "#F8FAFC",
  },
  header: {
    backgroundColor: "#1F2937",
    paddingBottom: 24,
    shadowColor: "#1F2937",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.2,
    shadowRadius: 12,
    elevation: 8,
  },
  headerContent: {
    flexDirection: "row",
    alignItems: "center",
    paddingHorizontal: 20,
    paddingTop: 16,
  },
  backButton: {
    width: 40,
    height: 40,
    borderRadius: 20,
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    justifyContent: "center",
    alignItems: "center",
    marginRight: 16,
  },
  headerTextContainer: {
    flex: 1,
  },
  headerTitle: {
    color: "#fff",
    fontSize: 24,
    fontWeight: "700",
    marginBottom: 4,
  },
  headerSubtitle: {
    color: "rgba(255, 255, 255, 0.8)",
    fontSize: 14,
    fontWeight: "400",
  },
  keyboardContainer: {
    flex: 1,
  },
  scrollContainer: {
    flex: 1,
    paddingTop: 8,
  },

  // Enhanced Verification Status
  verificationSection: {
    marginHorizontal: 20,
    marginBottom: 24,
  },
  verificationCard: {
    borderRadius: 20,
    overflow: "hidden",
  },
  verificationGradient: {
    padding: 24,
    borderRadius: 20,
    borderWidth: 2,
    shadowOffset: { width: 0, height: 8 },
    shadowOpacity: 0.15,
    shadowRadius: 16,
    elevation: 8,
  },
  verificationContent: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 16,
  },
  verificationIconContainer: {
    width: 48,
    height: 48,
    borderRadius: 24,
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    justifyContent: "center",
    alignItems: "center",
    marginRight: 16,
  },
  verificationTextContainer: {
    flex: 1,
  },
  verificationTitle: {
    fontSize: 18,
    fontWeight: "700",
    marginBottom: 4,
  },
  verificationSubtitle: {
    fontSize: 14,
    fontWeight: "500",
  },
  verificationLevelIndicator: {
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 12,
  },
  verificationLevel: {
    fontSize: 12,
    fontWeight: "700",
    letterSpacing: 0.5,
  },
  progressBarContainer: {
    marginBottom: 16,
  },
  progressBar: {
    height: 4,
    backgroundColor: "rgba(255, 255, 255, 0.3)",
    borderRadius: 2,
    overflow: "hidden",
  },
  progressFill: {
    height: "100%",
    borderRadius: 2,
  },
  upgradeButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    paddingVertical: 12,
    paddingHorizontal: 16,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: "rgba(255, 255, 255, 0.3)",
    gap: 8,
  },
  upgradeButtonText: {
    fontSize: 14,
    fontWeight: "600",
  },

  // Sections
  section: {
    marginBottom: 20,
  },
  sectionHeader: {
    flexDirection: "row",
    alignItems: "center",
    paddingHorizontal: 20,
    paddingBottom: 12,
  },
  sectionTitleDot: {
    width: 4,
    height: 16,
    borderRadius: 2,
    marginRight: 12,
  },
  sectionTitle: {
    fontSize: 10,
    fontWeight: "700",
    textTransform: "uppercase",
    letterSpacing: 1,
  },

  // Form Container and Input Groups
  formContainer: {
    backgroundColor: "#fff",
    marginHorizontal: 20,
    borderRadius: 16,
    padding: 20,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.04,
    shadowRadius: 8,
    elevation: 2,
  },
  settingsContainer: {
    marginHorizontal: 20,
  },
  inputGroup: {
    marginBottom: 16,
  },
  halfWidth: {
    flex: 1,
  },
  inputLabel: {
    fontSize: 14,
    fontWeight: "600",
    color: "#374151",
    marginBottom: 8,
  },
  inputContainer: {
    position: "relative",
  },
  textInput: {
    fontSize: 16,
    color: "#1F2937",
    paddingVertical: 14,
    paddingHorizontal: 16,
    backgroundColor: "#F9FAFB",
    borderRadius: 8,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  textInputError: {
    borderColor: "#EF4444",
    backgroundColor: "#FEF2F2",
  },
  passwordInput: {
    paddingRight: 50,
  },
  eyeIcon: {
    position: "absolute",
    right: 16,
    top: 17,
  },
  errorText: {
    color: "#EF4444",
    fontSize: 12,
    marginTop: 4,
    marginLeft: 4,
  },
  row: {
    flexDirection: "row",
    gap: 16,
    marginBottom: 0,
  },

  // Password Strength
  passwordStrength: {
    backgroundColor: "#F8FAFC",
    padding: 12,
    borderRadius: 8,
    marginTop: 8,
  },
  passwordStrengthTitle: {
    fontSize: 12,
    fontWeight: "600",
    color: "#374151",
    marginBottom: 8,
  },
  requirementsList: {
    gap: 2,
  },
  requirement: {
    fontSize: 11,
    lineHeight: 16,
  },
  requirementMet: {
    color: "#10B981",
  },
  requirementUnmet: {
    color: "#9CA3AF",
  },

  // Toggle Card
  toggleCard: {
    backgroundColor: "#fff",
    borderTopLeftRadius: 16,
    borderTopRightRadius: 16,
    padding: 16,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.04,
    shadowRadius: 8,
    elevation: 2,
  },
  toggleContent: {
    flexDirection: "row",
    alignItems: "center",
  },
  iconCircle: {
    width: 44,
    height: 44,
    borderRadius: 12,
    justifyContent: "center",
    alignItems: "center",
    marginRight: 16,
  },
  toggleText: {
    flex: 1,
  },
  toggleTitle: {
    fontSize: 16,
    fontWeight: "600",
    color: "#1F2937",
    marginBottom: 2,
  },
  toggleSubtitle: {
    fontSize: 13,
    color: "#6B7280",
    lineHeight: 18,
  },
  deleteCard: {
    backgroundColor: "#fff",
    borderBottomLeftRadius: 16,
    borderBottomRightRadius: 16,
    padding: 12,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.04,
    shadowRadius: 8,
    elevation: 2,
  },

  // Bottom spacing and buttons
  bottomSpacing: {
    height: 100,
  },
  buttonContainer: {
    paddingHorizontal: 20,
    paddingVertical: 16,
    backgroundColor: "#fff",
    borderTopWidth: 1,
    borderTopColor: "#E5E7EB",
  },
  saveButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    backgroundColor: "#D97706",
    paddingVertical: 16,
    borderRadius: 12,
    shadowColor: "#D97706",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.2,
    shadowRadius: 4,
    elevation: 3,
  },
  saveButtonDisabled: {
    backgroundColor: "#9CA3AF",
    shadowOpacity: 0,
  },
  saveButtonText: {
    color: "#fff",
    fontSize: 16,
    fontWeight: "600",
  },
  deleteButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    backgroundColor: "#FEF2F2",
    paddingVertical: 16,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: "#FECACA",
  },
  deleteButtonText: {
    color: "#DC2626",
    fontSize: 16,
    fontWeight: "600",
  },
  buttonIcon: {
    marginRight: 8,
  },
});

export default AccountManagement;

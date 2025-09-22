// components/UpgradeVerificationModal.js
import { Ionicons } from "@expo/vector-icons";
import * as DocumentPicker from "expo-document-picker"; // ✅ fix spelling
import React, { useState } from "react";
import {
  ActionSheetIOS,
  Alert,
  Image,
  Modal,
  Platform,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from "react-native";

const UpgradeVerificationModal = ({
  visible,
  onClose,
  onSelectLevel,
  verificationLevel = "unverified",
}) => {
  const [uploadedImages, setUploadedImages] = useState({});
  const [isProcessing, setIsProcessing] = useState(false);
  const [salaryAmount, setSalaryAmount] = useState("");
  const [salaryPeriod, setSalaryPeriod] = useState("yearly");
  const [uploadedDocs, setUploadedDocs] = useState({}); // { [action]: { uri, name, size, mimeType } }

  const salaryValue = Number(salaryAmount.replace(/[^\d]/g, ""));
  const salaryValid = salaryValue > 0;
  const pdfFile = uploadedDocs.upload_pay_pdf;
  const goldComplete = salaryValid && !!pdfFile;

  const tiers = [
    {
      level: "bronze",
      title: "Bronze Verification",
      description: "Basic verification with email confirmation.",
      icon: "medal-outline",
      color: "#B87333",
      requirements: ["Confirm your email address", "Verify phone number"],
      actions: [
        { label: "Verify Email", action: "verify_email" },
        { label: "Verify Phone", action: "verify_phone" },
      ],
    },
    {
      level: "silver",
      title: "Silver Verification",
      description: "Identity verification with government ID.",
      icon: "star-outline",
      color: "#C0C0C0",
      requirements: [
        "Upload government-issued ID",
        "Take a selfie for identity verification",
      ],
      actions: [
        { label: "Upload ID", action: "upload_id" },
        { label: "Take Selfie", action: "take_selfie" },
      ],
    },

    {
      level: "gold",
      title: "Gold Verification",
      description: "Financial verification for trusted renting.",
      icon: "trophy-outline",
      color: "#FFD700",
      requirements: [
        "Enter your salary (monthly or yearly)",
        "Upload a PDF of your pay document",
      ],
      actions: [
        { label: "Enter Salary", action: "enter_salary" },
        { label: "Upload Pay PDF", action: "upload_pay_pdf" },
      ],
    },

    {
      level: "platinum",
      title: "Platinum Verification",
      description: "Full trust verification including background check.",
      icon: "diamond-outline",
      color: "#E5E4E2",
      requirements: [
        "Background check consent",
        "Reference contacts",
        "Additional identity verification",
      ],
      actions: [
        { label: "Consent Background Check", action: "background_check" },
        { label: "Add References", action: "add_references" },
        { label: "Enhanced ID Check", action: "enhanced_id" },
      ],
    },
  ];

  // Get the next verification level based on current level
  const getNextTier = () => {
    // const levelOrder = ["unverified", "bronze", "silver", "gold", "platinum"];
    const levelOrder = ["unverified", "bronze", "silver", "gold"];
    const currentIndex = levelOrder.indexOf(verificationLevel);

    if (currentIndex === -1 || currentIndex >= levelOrder.length - 1) {
      return null;
    }
    const nextLevel = levelOrder[currentIndex + 1];
    return tiers.find((tier) => tier.level === nextLevel);
  };

  const nextTier = getNextTier();

  // Request camera permissions
  const requestCameraPermissions = async () => {
    const { status } = await ImagePicker.requestCameraPermissionsAsync();
    if (status !== "granted") {
      Alert.alert(
        "Permission needed",
        "Camera permission is required to take photos."
      );
      return false;
    }
    return true;
  };

  const pickPdf = async () => {
    try {
      const res = await DocumentPicker.getDocumentAsync({
        type: "application/pdf",
        copyToCacheDirectory: true,
        multiple: false,
      });

      if (res.canceled) return null;

      const file = res.assets?.[0];
      if (!file) return null;

      if (file.mimeType !== "application/pdf") {
        Alert.alert("Invalid file", "Please select a PDF file.");
        return null;
      }

      // Optional size gate (e.g., 10MB)
      const MAX_BYTES = 10 * 1024 * 1024;
      if (file.size && file.size > MAX_BYTES) {
        Alert.alert("File too large", "Please upload a PDF under 10 MB.");
        return null;
      }

      return file; // { uri, name, size, mimeType }
    } catch (e) {
      Alert.alert("Error", "Failed to pick a PDF. Please try again.");
      return null;
    }
  };

  // Request media library permissions
  const requestMediaLibraryPermissions = async () => {
    const { status } = await ImagePicker.requestMediaLibraryPermissionsAsync();
    if (status !== "granted") {
      Alert.alert(
        "Permission needed",
        "Photo library permission is required to select images."
      );
      return false;
    }
    return true;
  };

  // Show action sheet for image selection
  const showImagePicker = (actionType) => {
    const options = ["Take Photo", "Choose from Photos", "Cancel"];
    const cancelButtonIndex = 2;

    if (Platform.OS === "ios") {
      ActionSheetIOS.showActionSheetWithOptions(
        {
          options,
          cancelButtonIndex,
          title: "Select Image",
        },
        (buttonIndex) => {
          if (buttonIndex === 0) {
            takePicture(actionType);
          } else if (buttonIndex === 1) {
            pickImageFromLibrary(actionType);
          }
        }
      );
    } else {
      // For Android, show custom modal or use Alert
      Alert.alert("Select Image", "Choose an option", [
        { text: "Take Photo", onPress: () => takePicture(actionType) },
        {
          text: "Choose from Photos",
          onPress: () => pickImageFromLibrary(actionType),
        },
        { text: "Cancel", style: "cancel" },
      ]);
    }
  };

  // Take picture with camera - unified for all image capture needs
  const takePicture = async (actionType) => {
    const hasPermission = await requestCameraPermissions();
    if (!hasPermission) return;

    setIsProcessing(true);
    try {
      // Configure camera options based on action type
      const cameraOptions = {
        mediaTypes:  [MediaType.Image],
        allowsEditing: true,
        aspect: actionType === "upload_id" ? [4, 3] : [1, 1], // Square for selfies, 4:3 for IDs
        quality: 0.8,
      };

      // Set camera type for selfies
      if (actionType === "take_selfie") {
        cameraOptions.cameraType = ImagePicker.CameraType.front;
      }

      const result = await ImagePicker.launchCameraAsync(cameraOptions);

      if (!result.canceled && result.assets[0]) {
        setUploadedImages((prev) => ({
          ...prev,
          [actionType]: result.assets[0].uri,
        }));

        const actionName = actionType === "take_selfie" ? "Selfie" : "Image";
        Alert.alert("Success", `${actionName} captured successfully!`);
      }
    } catch (error) {
      Alert.alert("Error", "Failed to take picture. Please try again.");
    }
    setIsProcessing(false);
  };

  // Pick image from library
  const pickImageFromLibrary = async (actionType) => {
    const hasPermission = await requestMediaLibraryPermissions();
    if (!hasPermission) return;

    setIsProcessing(true);
    try {
      const result = await ImagePicker.launchImageLibraryAsync({
        mediaTypes:  [MediaType.Image],
        allowsEditing: true,
        aspect: actionType === "upload_id" ? [4, 3] : [1, 1],
        quality: 0.8,
      });

      if (!result.canceled && result.assets[0]) {
        setUploadedImages((prev) => ({
          ...prev,
          [actionType]: result.assets[0].uri,
        }));
        Alert.alert("Success", "Image selected successfully!");
      }
    } catch (error) {
      Alert.alert("Error", "Failed to select image. Please try again.");
    }
    setIsProcessing(false);
  };

  const handleAction = (actionType) => {
    switch (actionType) {
      case "verify_email":
        Alert.alert(
          "Email Verification",
          "A verification link has been sent to your email address."
        );
        break;
      case "verify_phone":
        Alert.alert(
          "Phone Verification",
          "Please enter the SMS code sent to your phone."
        );
        break;
      case "upload_id":
      case "take_selfie":
      case "upload_income":
      case "enhanced_id":
        showImagePicker(actionType);
        break;
      case "connect_bank":
        Alert.alert(
          "Connect Bank",
          "You will be redirected to securely connect your bank account."
        );
        break;
      case "upload_pay_pdf": {
        (async () => {
          const file = await pickPdf();
          if (!file) return;
          setUploadedDocs((prev) => ({ ...prev, upload_pay_pdf: file }));
          Alert.alert("Success", "PDF uploaded successfully!");
        })();
        break;
      }
      case "credit_check":
        Alert.alert(
          "Credit Check",
          "You will be asked to authorize a soft credit check."
        );
        break;
      case "background_check":
        Alert.alert(
          "Background Check",
          "Please provide consent for a background check."
        );
        break;
      case "add_references":
        Alert.alert(
          "References",
          "Please add 2-3 personal or professional references."
        );
        break;
      default:
        Alert.alert(
          "Coming Soon",
          "This verification step will be available soon."
        );
    }
  };

  if (!nextTier) {
    return (
      <Modal visible={visible} animationType="slide" transparent>
        <View style={styles.overlay}>
          <View style={styles.container}>
            <View style={styles.header}>
              <Text style={styles.title}>Verification Complete</Text>
              <TouchableOpacity onPress={onClose}>
                <Ionicons name="close" size={24} color="#374151" />
              </TouchableOpacity>
            </View>
            <View style={styles.completedContainer}>
              <Ionicons name="checkmark-circle" size={64} color="#10B981" />
              <Text style={styles.completedTitle}>
                You're at the highest verification level!
              </Text>
              <Text style={styles.completedDescription}>
                You have achieved Platinum verification status.
              </Text>
            </View>
          </View>
        </View>
      </Modal>
    );
  }

  return (
    <Modal visible={visible} animationType="slide" transparent>
      <View style={styles.overlay}>
        <View style={styles.container}>
          <View style={styles.header}>
            <Text style={styles.title}>Upgrade to {nextTier.title}</Text>
            <TouchableOpacity onPress={onClose}>
              <Ionicons name="close" size={24} color="#374151" />
            </TouchableOpacity>
          </View>

          <ScrollView showsVerticalScrollIndicator={false}>
            <View style={styles.nextLevelCard}>
              <View style={styles.cardHeader}>
                <Ionicons
                  name={nextTier.icon}
                  size={32}
                  color={nextTier.color}
                  style={styles.icon}
                />
                <View style={{ flex: 1 }}>
                  <Text
                    style={[styles.nextLevelTitle, { color: nextTier.color }]}
                  >
                    {nextTier.title}
                  </Text>
                  <Text style={styles.nextLevelDescription}>
                    {nextTier.description}
                  </Text>
                </View>
              </View>

              <View style={styles.requirementsSection}>
                <Text style={styles.sectionTitle}>
                  Requirements to Complete:
                </Text>
                {nextTier.requirements.map((requirement, index) => (
                  <View key={index} style={styles.requirementItem}>
                    <Ionicons
                      name="checkmark-circle-outline"
                      size={20}
                      color="#10B981"
                    />
                    <Text style={styles.requirementText}>{requirement}</Text>
                  </View>
                ))}
              </View>

              <View style={styles.actionsSection}>
                <Text style={styles.sectionTitle}>Complete Verification:</Text>

                {nextTier.level === "gold" ? (
                  <>
                    {/* SALARY CARD */}
                    <View
                      style={[styles.card, salaryValid && styles.cardCompleted]}
                    >
                      <View style={styles.cardHeaderRow}>
                        <Text style={styles.cardTitle}>Enter Salary</Text>
                        <View
                          style={[
                            styles.badge,
                            salaryValid
                              ? styles.badgeDone
                              : styles.badgePending,
                          ]}
                        >
                          <Text
                            style={[
                              styles.badgeText,
                              salaryValid && styles.badgeTextDone,
                            ]}
                          >
                            {salaryValid ? "Done" : "Pending"}
                          </Text>
                        </View>
                      </View>

                      <Text style={styles.cardHint}>
                        Provide your salary and select the period.
                      </Text>

                      <View style={styles.salaryRow}>
                        <View style={styles.salaryInputWrap}>
                          <Text style={styles.currencyPrefix}>$</Text>
                          <TextInput
                            value={salaryAmount}
                            onChangeText={(t) =>
                              setSalaryAmount(t.replace(/[^\d]/g, ""))
                            }
                            keyboardType="number-pad"
                            placeholder="0"
                            style={styles.salaryInput}
                          />
                          <Text style={styles.suffixText}>
                            {salaryPeriod === "monthly" ? "/mo" : "/yr"}
                          </Text>
                        </View>

                        <View style={styles.periodToggle}>
                          <TouchableOpacity
                            style={[
                              styles.periodButton,
                              salaryPeriod === "monthly" &&
                                styles.periodButtonActive,
                            ]}
                            onPress={() => setSalaryPeriod("monthly")}
                          >
                            <Text
                              style={[
                                styles.periodButtonText,
                                salaryPeriod === "monthly" &&
                                  styles.periodButtonTextActive,
                              ]}
                            >
                              Monthly
                            </Text>
                          </TouchableOpacity>
                          <TouchableOpacity
                            style={[
                              styles.periodButton,
                              salaryPeriod === "yearly" &&
                                styles.periodButtonActive,
                            ]}
                            onPress={() => setSalaryPeriod("yearly")}
                          >
                            <Text
                              style={[
                                styles.periodButtonText,
                                salaryPeriod === "yearly" &&
                                  styles.periodButtonTextActive,
                              ]}
                            >
                              Yearly
                            </Text>
                          </TouchableOpacity>
                        </View>
                      </View>

                      {!!salaryAmount && !salaryValid && (
                        <Text style={styles.errorText}>
                          Enter a valid amount greater than 0.
                        </Text>
                      )}
                    </View>

                    {/* PDF CARD */}
                    <View
                      style={[styles.card, pdfFile && styles.cardCompleted]}
                    >
                      <View style={styles.cardHeaderRow}>
                        <Text style={styles.cardTitle}>Upload Pay PDF</Text>
                        <View
                          style={[
                            styles.badge,
                            pdfFile ? styles.badgeDone : styles.badgePending,
                          ]}
                        >
                          <Text
                            style={[
                              styles.badgeText,
                              pdfFile && styles.badgeTextDone,
                            ]}
                          >
                            {pdfFile ? "Done" : "Pending"}
                          </Text>
                        </View>
                      </View>

                      <Text style={styles.cardHint}>
                        Upload a recent pay document (PDF).
                      </Text>

                      {pdfFile ? (
                        <View style={styles.pdfPreview}>
                          <Ionicons
                            name="document-text-outline"
                            size={24}
                            color="#374151"
                            style={{ marginRight: 12 }}
                          />
                          <View style={{ flex: 1 }}>
                            <Text style={styles.pdfName} numberOfLines={1}>
                              {pdfFile.name || "PDF attached"}
                            </Text>
                            {!!pdfFile.size && (
                              <Text style={styles.pdfMeta}>
                                {(pdfFile.size / (1024 * 1024)).toFixed(2)} MB
                              </Text>
                            )}
                          </View>
                          <TouchableOpacity
                            style={styles.retakeButton}
                            onPress={() => handleAction("upload_pay_pdf")}
                          >
                            <Text style={styles.retakeButtonText}>Replace</Text>
                          </TouchableOpacity>
                        </View>
                      ) : (
                        <TouchableOpacity
                          style={styles.pdfUploadBtn}
                          onPress={() => handleAction("upload_pay_pdf")}
                        >
                          <Ionicons
                            name="cloud-upload-outline"
                            size={18}
                            color="#111827"
                          />
                          <Text style={styles.pdfUploadText}>Choose PDF</Text>
                        </TouchableOpacity>
                      )}
                    </View>
                  </>
                ) : (
                  /* NON‑SILVER FALLBACK: keep your original mapped actions & previews */
                  <>
                    {nextTier.actions.map((action, index) => {
                      const isDone = !!uploadedImages[action.action];
                      return (
                        <View key={index}>
                          <TouchableOpacity
                            style={[
                              styles.actionButton,
                              { borderColor: nextTier.color },
                              isDone && styles.completedActionButton,
                            ]}
                            onPress={() => handleAction(action.action)}
                            disabled={isProcessing}
                          >
                            <View style={styles.actionButtonContent}>
                              <Text
                                style={[
                                  styles.actionButtonText,
                                  {
                                    color: isDone ? "#10B981" : nextTier.color,
                                  },
                                ]}
                              >
                                {action.label}
                              </Text>
                              {isDone && (
                                <Ionicons
                                  name="checkmark-circle"
                                  size={18}
                                  color="#10B981"
                                  style={{ marginLeft: 8 }}
                                />
                              )}
                            </View>

                            <Ionicons
                              name={isDone ? "checkmark" : "arrow-forward"}
                              size={18}
                              color={isDone ? "#10B981" : nextTier.color}
                            />
                          </TouchableOpacity>

                          {uploadedImages[action.action] && (
                            <View style={styles.imagePreview}>
                              <Image
                                source={{ uri: uploadedImages[action.action] }}
                                style={styles.previewImage}
                              />
                              <TouchableOpacity
                                style={styles.retakeButton}
                                onPress={() => handleAction(action.action)}
                              >
                                <Text style={styles.retakeButtonText}>
                                  Retake
                                </Text>
                              </TouchableOpacity>
                            </View>
                          )}
                        </View>
                      );
                    })}
                  </>
                )}
              </View>

              <TouchableOpacity
                style={[
                  styles.startButton,
                  { backgroundColor: nextTier.color },
                ]}
                onPress={() => onSelectLevel(nextTier.level)}
              >
                <Text style={styles.startButtonText}>
                  Start {nextTier.title}
                </Text>
              </TouchableOpacity>
            </View>
          </ScrollView>
        </View>
      </View>
    </Modal>
  );
};

const styles = StyleSheet.create({
  overlay: {
    flex: 1,
    backgroundColor: "rgba(0,0,0,0.4)",
    justifyContent: "flex-end",
  },
  container: {
    backgroundColor: "#fff",
    borderTopLeftRadius: 16,
    borderTopRightRadius: 16,
    padding: 20,
    maxHeight: "85%",
  },
  header: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 20,
  },
  title: {
    fontSize: 20,
    fontWeight: "700",
    color: "#111827",
  },
  nextLevelCard: {
    backgroundColor: "#F9FAFB",
    borderRadius: 16,
    padding: 20,
  },
  cardHeader: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 20,
  },
  icon: {
    marginRight: 16,
  },
  nextLevelTitle: {
    fontSize: 18,
    fontWeight: "700",
  },
  nextLevelDescription: {
    fontSize: 14,
    color: "#6B7280",
    marginTop: 4,
  },
  requirementsSection: {
    marginBottom: 24,
  },
  sectionTitle: {
    fontSize: 16,
    fontWeight: "600",
    color: "#111827",
    marginBottom: 12,
  },
  requirementItem: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 8,
  },
  requirementText: {
    fontSize: 14,
    color: "#374151",
    marginLeft: 8,
    flex: 1,
  },
  actionsSection: {
    marginBottom: 24,
  },
  actionButton: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    borderWidth: 1.5,
    borderRadius: 12,
    padding: 16,
    marginBottom: 8,
    backgroundColor: "#fff",
  },
  completedActionButton: {
    borderColor: "#10B981",
    backgroundColor: "#F0FDF4",
  },
  actionButtonContent: {
    flexDirection: "row",
    alignItems: "center",
    flex: 1,
  },
  actionButtonText: {
    fontSize: 15,
    fontWeight: "600",
    marginRight: 8,
  },
  imagePreview: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#F3F4F6",
    borderRadius: 8,
    padding: 12,
    marginBottom: 8,
  },
  previewImage: {
    width: 60,
    height: 60,
    borderRadius: 8,
    marginRight: 12,
  },
  retakeButton: {
    backgroundColor: "#6B7280",
    borderRadius: 6,
    paddingHorizontal: 12,
    paddingVertical: 6,
  },
  retakeButtonText: {
    color: "#fff",
    fontSize: 12,
    fontWeight: "600",
  },
  startButton: {
    borderRadius: 12,
    padding: 16,
    alignItems: "center",
  },
  startButtonText: {
    fontSize: 16,
    fontWeight: "700",
    color: "#fff",
  },
  completedContainer: {
    alignItems: "center",
    padding: 40,
  },
  completedTitle: {
    fontSize: 18,
    fontWeight: "700",
    color: "#111827",
    marginTop: 16,
    marginBottom: 8,
  },
  completedDescription: {
    fontSize: 14,
    color: "#6B7280",
    textAlign: "center",
  },
  salaryForm: {
    backgroundColor: "#F3F4F6",
    borderRadius: 8,
    padding: 12,
    marginBottom: 8,
  },
  salaryLabel: {
    fontSize: 12,
    color: "#374151",
    marginBottom: 8,
    fontWeight: "600",
  },
  salaryRow: { flexDirection: "row", alignItems: "center" },
  salaryInputWrap: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#fff",
    borderWidth: 1,
    borderColor: "#D1D5DB",
    borderRadius: 8,
    paddingHorizontal: 8,
    height: 40,
    flex: 1,
    marginRight: 8,
  },
  currencyPrefix: { color: "#6B7280", marginRight: 4, fontWeight: "700" },
  salaryInput: { flex: 1, height: "100%" },
  periodToggle: {
    flexDirection: "row",
    backgroundColor: "#E5E7EB",
    borderRadius: 8,
    overflow: "hidden",
  },
  periodButton: { paddingHorizontal: 12, paddingVertical: 8 },
  periodButtonActive: { backgroundColor: "#111827" },
  periodButtonText: { color: "#111827", fontWeight: "600" },
  periodButtonTextActive: { color: "#fff" },
  salaryActions: {
    flexDirection: "row",
    justifyContent: "flex-end",
    marginTop: 8,
  },
  salaryBtn: {
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 6,
    marginLeft: 8,
  },
  salaryCancel: { backgroundColor: "#E5E7EB" },
  salarySave: { backgroundColor: "#10B981" },
  salaryCancelText: { color: "#111827", fontWeight: "700" },
  salarySaveText: { color: "#fff", fontWeight: "700" },
  pdfPreview: {
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#F3F4F6",
    borderRadius: 8,
    padding: 12,
    marginBottom: 8,
  },
  pdfName: { fontSize: 14, color: "#111827", fontWeight: "600" },
  pdfMeta: { fontSize: 12, color: "#6B7280", marginTop: 2 },
  card: {
    backgroundColor: "#fff",
    borderRadius: 12,
    padding: 16,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: "#E5E7EB",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.08,
    shadowRadius: 4,
    elevation: 2,
  },
  cardCompleted: {
    borderColor: "#10B981",
    backgroundColor: "#F0FDF4",
    elevation: 0,
  },
  cardHeaderRow: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    marginBottom: 8,
  },
  cardTitle: { fontSize: 15, fontWeight: "700", color: "#111827" },
  cardHint: { fontSize: 13, color: "#6B7280", marginBottom: 12 },

  badge: {
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 999,
    borderWidth: 1,
  },
  badgePending: { borderColor: "#D1D5DB", backgroundColor: "#F9FAFB" },
  badgeDone: { borderColor: "#10B981", backgroundColor: "#D1FAE5" },
  badgeText: { fontSize: 11, color: "#374151", fontWeight: "700" },
  badgeTextDone: { color: "#065F46" },

  suffixText: { color: "#6B7280", marginLeft: 6, fontWeight: "700" },

  pdfUploadBtn: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    height: 44,
    borderRadius: 10,
    borderWidth: 1.5,
    borderColor: "#D1D5DB",
    backgroundColor: "#F9FAFB",
  },
  pdfUploadText: { marginLeft: 8, fontWeight: "700", color: "#111827" },

  errorText: {
    color: "#DC2626",
    marginTop: 8,
    fontSize: 12,
    fontWeight: "600",
  },
});

export default UpgradeVerificationModal;

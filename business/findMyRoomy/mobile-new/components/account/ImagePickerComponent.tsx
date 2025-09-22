import React, { useState } from "react";
import {
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
  Image,
  ScrollView,
  Dimensions,
  Modal,
  Pressable,
} from "react-native";
import { Ionicons } from "@expo/vector-icons";

import * as ImagePicker from "expo-image-picker"
import authService from "@/database/authService";

const MAX_PHOTOS = 6;
const SCREEN_WIDTH = Dimensions.get("window").width;

const ImagePickerComponent = () => {
  const [photos, setPhotos] = useState<string[]>([]);
  const [selectedPhoto, setSelectedPhoto] = useState<string | null>(null);

  const addPhoto = async () => {
    try {
      let result = await ImagePicker.launchImageLibraryAsync({
        mediaTypes: ['images'],
        allowsEditing: true,
        aspect: [4, 3],
        quality: 1,
      });
  
      if (!result.canceled && result.assets && result.assets.length > 0) {
        const asset = result.assets[0];
        console.log("Selected asset:", asset);
        
        // Upload to Supabase
        const uploadResult = await authService.uploadUserPhoto(asset.uri, {
          bucket: "user-photos",
          makePublic: false,
          contentType: asset.mimeType || 'image/jpeg',
        });
  
        console.log("Upload result:", uploadResult);
        console.log("The uri", asset.uri)
  
        if (uploadResult.success && uploadResult.url) {
          const url = uploadResult.url; // now inferred as string
          setPhotos((prev) => [...prev, url]);
          console.log("✅ Photo added to UI with URL:", url);
        } else {
          console.error("❌ Upload failed:", uploadResult.error);
        }
        
      }
    } catch (error) {
      console.error("❌ Error in addPhoto:", error);
    }
  };

  const removePhoto = (index: number) => {
    setPhotos((prev) => prev.filter((_, i) => i !== index));
  };

  return (
    <View style={styles.container}>
      <ScrollView contentContainerStyle={styles.grid}>
        {photos.map((uri, index) => (
          <View key={index} style={styles.photoBox}>
            <Pressable onLongPress={() => setSelectedPhoto(uri)}>
              <Image source={{ uri }} style={styles.photoImage} />
            </Pressable>

            {/* Red X button */}
            <TouchableOpacity
              style={styles.removeButton}
              onPress={() => removePhoto(index)}
            >
              <Ionicons name="close" size={16} color="#fff" />
            </TouchableOpacity>
          </View>
        ))}

        {photos.length < MAX_PHOTOS && (
          <TouchableOpacity style={styles.addPhotoButton} onPress={addPhoto}>
            <Ionicons name="add" size={32} color="#007AFF" />
            <Text style={styles.addPhotoText}>Add</Text>
          </TouchableOpacity>
        )}
      </ScrollView>

      {/* Modal for enlarged photo */}
      <Modal visible={!!selectedPhoto} transparent animationType="fade">
        <View style={styles.modalBackground}>
          <Pressable style={styles.modalBackground} onPress={() => setSelectedPhoto(null)}>
            {selectedPhoto && (
              <Image
                source={{ uri: selectedPhoto }}
                style={styles.fullImage}
                resizeMode="contain"
              />
            )}
          </Pressable>

          {/* Close button */}
          <TouchableOpacity
            style={styles.modalCloseButton}
            onPress={() => setSelectedPhoto(null)}
          >
            <Ionicons name="close" size={28} color="#fff" />
          </TouchableOpacity>
        </View>
      </Modal>
    </View>
  );
};

const BOX_SIZE = (SCREEN_WIDTH - 20 * 3 - 12 * 2) / 3;

const styles = StyleSheet.create({
  container: {
    padding: 20,
    backgroundColor: "#F5F5F5",
    flex: 1,
  },
  grid: {
    flexDirection: "row",
    flexWrap: "wrap",
    justifyContent: "flex-start",
  },
  photoBox: {
    width: BOX_SIZE,
    height: BOX_SIZE,
    borderRadius: 12,
    margin: 6,
    overflow: "hidden",
    backgroundColor: "#E5E5E5",
  },
  photoImage: {
    width: "100%",
    height: "100%",
  },
  removeButton: {
    position: "absolute",
    top: 2,
    right: 2,
    backgroundColor: "red",
    borderRadius: 12,
    padding: 4,
    justifyContent: "center",
    alignItems: "center",
    zIndex: 1,
  },
  addPhotoButton: {
    width: BOX_SIZE,
    height: BOX_SIZE,
    borderRadius: 12,
    borderWidth: 2,
    borderColor: "#007AFF",
    borderStyle: "dashed",
    justifyContent: "center",
    alignItems: "center",
    backgroundColor: "#FFFFFF",
    margin: 6,
  },
  addPhotoText: {
    marginTop: 4,
    fontSize: 12,
    fontWeight: "600",
    color: "#007AFF",
  },
  modalBackground: {
    flex: 1,
    backgroundColor: "rgba(0,0,0,0.9)",
    justifyContent: "center",
    alignItems: "center",
  },
  fullImage: {
    width: "90%",
    height: "70%",
  },
  modalCloseButton: {
    position: "absolute",
    top: 40,
    right: 20,
    backgroundColor: "rgba(255,0,0,0.8)",
    padding: 8,
    borderRadius: 20,
  },
});

export default ImagePickerComponent;

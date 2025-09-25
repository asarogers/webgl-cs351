import React, { useState, useEffect } from "react";
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
  ActivityIndicator,
} from "react-native";
import { Ionicons } from "@expo/vector-icons";
import * as ImagePicker from "expo-image-picker";
import authService from "@/database/authService";

const MAX_PHOTOS = 6;
const SCREEN_WIDTH = Dimensions.get("window").width;

type Photo = { 
  id: string; // Add unique id
  uri: string; 
  path: string;
};

type Props = {
  editing?: boolean;
  profile?: any;
  toggleEdit?: () => void;
  setProfile: any;
  setDirty: any;
};

export const ImagePickerComponent: React.FC<Props> = ({
  setProfile, profile, setDirty
}) => {
  const [photos, setPhotos] = useState<Photo[]>([]);
  const [selectedPhoto, setSelectedPhoto] = useState<string | null>(null);
  const [isUploading, setIsUploading] = useState(false);
  const [deletingIds, setDeletingIds] = useState<Set<string>>(new Set());
  const [loadingIds, setLoadingIds] = useState<Set<string>>(new Set());

  // 🚀 Load photos on mount
  useEffect(() => {
    const fetchPhotos = async () => {
      try {
        const result = await authService.getUserPhotos({ bucket: "user-photos" });
  
        if (result.success && result.paths) {
          const photoObjects = result.photos.map((uri, i) => ({
            id: `photo-${Date.now()}-${i}`, // Generate unique IDs
            uri: uri,
            path: result.paths[i],
          }));
          
          setPhotos(photoObjects);
        } else {
          console.error("❌ Failed to fetch photos:", result.error);
        }
      } catch (error) {
        console.error("❌ Error loading photos:", error);
      }
    };
  
    fetchPhotos();
  }, []);

  // ✅ Sync to parent whenever photos changes
  useEffect(() => {
    setProfile((prev: any) => ({
      ...prev,
      photos,
    }));
  }, [photos]);

  const addPhoto = async () => {
    try {
      setIsUploading(true);
      const result = await ImagePicker.launchImageLibraryAsync({
        mediaTypes: ["images"],
        allowsEditing: true,
        aspect: [4, 3],
        quality: 1,
      });
  
      if (!result.canceled && result.assets?.length > 0) {
        const asset = result.assets[0];
        const uploadResult = await authService.uploadUserPhoto(asset.uri, {
          bucket: "user-photos",
          contentType: asset.mimeType || "image/jpeg",
        });
  
        if (uploadResult.success && uploadResult.url) {
          const newPhoto = { 
            id: `photo-${Date.now()}`, // Generate unique ID
            uri: uploadResult.url,
            path: uploadResult.path,
          };
          
          setPhotos((prev) => [...prev, newPhoto]);
          setDirty(true);
        } else {
          console.error("❌ Upload failed:", uploadResult.error);
        }
      }
    } catch (error) {
      console.error("❌ Error in addPhoto:", error);
    } finally {
      setIsUploading(false);
    }
  };

  const removePhoto = async (photoId: string) => {
    try {
      setDeletingIds(prev => new Set([...prev, photoId]));
      
      const photoToDelete = photos.find(p => p.id === photoId);
      if (!photoToDelete) return;
      
      const deleteResult = await authService.deleteUserPhoto(photoToDelete.path);

      if (deleteResult.success) {
        setPhotos((prev) => prev.filter((p) => p.id !== photoId));
        // Clean up any loading states for this photo
        setLoadingIds(prev => {
          const newSet = new Set([...prev]);
          newSet.delete(photoId);
          return newSet;
        });
      } else {
        console.error("❌ Delete failed:", deleteResult.error);
      }
    } catch (error) {
      console.error("❌ Error deleting photo:", error);
    } finally {
      setDeletingIds(prev => {
        const newSet = new Set([...prev]);
        newSet.delete(photoId);
        return newSet;
      });
    }
  };

  const handleImageLoadStart = (photoId: string) =>
    setLoadingIds((prev) => new Set([...prev, photoId]));

  const handleImageLoadEnd = (photoId: string) =>
    setLoadingIds((prev) => {
      const newSet = new Set([...prev]);
      newSet.delete(photoId);
      return newSet;
    });

  return (
    <View style={styles.container}>
      <ScrollView contentContainerStyle={styles.grid}>
        {photos.map((photo) => (
          <View key={photo.id} style={styles.photoBox}>
            <Pressable onLongPress={() => setSelectedPhoto(photo.uri)}>
              <Image
                source={{ uri: photo.uri }}
                style={styles.photoImage}
                onLoadStart={() => handleImageLoadStart(photo.id)}
                onLoadEnd={() => handleImageLoadEnd(photo.id)}
                onError={() => handleImageLoadEnd(photo.id)}
              />
              {loadingIds.has(photo.id) && (
                <View style={styles.imageLoadingOverlay}>
                  <ActivityIndicator size="small" color="#007AFF" />
                </View>
              )}
              {deletingIds.has(photo.id) && (
                <View style={styles.deleteLoadingOverlay}>
                  <ActivityIndicator size="small" color="#fff" />
                </View>
              )}
            </Pressable>
            <TouchableOpacity
              style={[
                styles.removeButton,
                deletingIds.has(photo.id) && styles.removeButtonDisabled,
              ]}
              onPress={() => removePhoto(photo.id)}
              disabled={deletingIds.has(photo.id)}
            >
              {deletingIds.has(photo.id) ? (
                <ActivityIndicator size={12} color="#fff" />
              ) : (
                <Ionicons name="close" size={16} color="#fff" />
              )}
            </TouchableOpacity>
          </View>
        ))}
        {photos.length < MAX_PHOTOS && (
          <TouchableOpacity
            style={[
              styles.addPhotoButton,
              isUploading && styles.addPhotoButtonDisabled,
            ]}
            onPress={addPhoto}
            disabled={isUploading}
          >
            {isUploading ? (
              <>
                <ActivityIndicator size={32} color="#007AFF" />
                <Text style={styles.addPhotoText}>Uploading...</Text>
              </>
            ) : (
              <>
                <Ionicons name="add" size={32} color="#007AFF" />
                <Text style={styles.addPhotoText}>Add</Text>
              </>
            )}
          </TouchableOpacity>
        )}
      </ScrollView>
      <Modal visible={!!selectedPhoto} transparent animationType="fade">
        <View style={styles.modalBackground}>
          <Pressable
            style={styles.modalBackground}
            onPress={() => setSelectedPhoto(null)}
          >
            {selectedPhoto && (
              <Image
                source={{ uri: selectedPhoto }}
                style={styles.fullImage}
                resizeMode="contain"
              />
            )}
          </Pressable>
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
  container: { padding: 20, backgroundColor: "#F5F5F5", flex: 1 },
  grid: { flexDirection: "row", flexWrap: "wrap", justifyContent: "flex-start" },
  photoBox: {
    width: BOX_SIZE,
    height: BOX_SIZE,
    borderRadius: 12,
    margin: 6,
    overflow: "hidden",
    backgroundColor: "#E5E5E5",
    position: "relative",
  },
  photoImage: { width: "100%", height: "100%" },
  removeButton: {
    position: "absolute",
    top: 2,
    right: 2,
    backgroundColor: "red",
    borderRadius: 12,
    padding: 4,
    justifyContent: "center",
    alignItems: "center",
    zIndex: 2,
    minWidth: 24,
    minHeight: 24,
  },
  removeButtonDisabled: { backgroundColor: "rgba(255, 0, 0, 0.5)" },
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
  addPhotoButtonDisabled: {
    borderColor: "#B0B0B0",
    backgroundColor: "#F8F8F8",
  },
  addPhotoText: {
    marginTop: 4,
    fontSize: 12,
    fontWeight: "600",
    color: "#007AFF",
  },
  imageLoadingOverlay: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: "rgba(255, 255, 255, 0.8)",
    justifyContent: "center",
    alignItems: "center",
    zIndex: 1,
  },
  deleteLoadingOverlay: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: "rgba(0, 0, 0, 0.5)",
    justifyContent: "center",
    alignItems: "center",
    zIndex: 1,
  },
  modalBackground: {
    flex: 1,
    backgroundColor: "rgba(0,0,0,0.9)",
    justifyContent: "center",
    alignItems: "center",
  },
  fullImage: { width: "90%", height: "70%" },
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
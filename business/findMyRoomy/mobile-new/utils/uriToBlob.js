import * as FileSystem from "expo-file-system";
export const uriToBlob = async (uri, mimeType = "image/jpeg") => {
    console.log("🔍 [uriToBlob] STEP 1: Starting conversion");
    console.log("🔍 [uriToBlob] Input URI:", uri);
    console.log("🔍 [uriToBlob] Expected MIME type:", mimeType);
    
    try {
      console.log("🔍 [uriToBlob] STEP 2: Calling fetch...");
      const response = await fetch(uri);
      
      console.log("🔍 [uriToBlob] STEP 3: Fetch response received");
      console.log("🔍 [uriToBlob] Response status:", response.status);
      console.log("🔍 [uriToBlob] Response ok:", response.ok);
      console.log("🔍 [uriToBlob] Response headers:", JSON.stringify([...response.headers.entries()]));
      
      if (!response.ok) {
        throw new Error(`Failed to fetch file: ${response.status} ${response.statusText}`);
      }
      
      console.log("🔍 [uriToBlob] STEP 4: Converting to blob...");
      const blob = await response.blob();
      
      console.log("🔍 [uriToBlob] STEP 5: Blob created successfully");
      console.log("🔍 [uriToBlob] Blob size:", blob.size, "bytes");
      console.log("🔍 [uriToBlob] Blob type:", blob.type);
      console.log("🔍 [uriToBlob] Blob constructor:", blob.constructor.name);
      
      // Ensure correct MIME type
      const finalBlob = new Blob([blob], { type: mimeType });
      console.log("🔍 [uriToBlob] STEP 6: Final blob with correct MIME type");
      console.log("🔍 [uriToBlob] Final blob size:", finalBlob.size);
      console.log("🔍 [uriToBlob] Final blob type:", finalBlob.type);
      
      return finalBlob;
    } catch (error) {
      console.error('❌ [uriToBlob] ERROR:', error);
      console.error('❌ [uriToBlob] Error name:', error.name);
      console.error('❌ [uriToBlob] Error message:', error.message);
      console.error('❌ [uriToBlob] Error stack:', error.stack);
      throw error;
    }
  };
  export const uriToUint8Array = async (uri) => {
    try {
      const response = await fetch(uri);
      if (!response.ok) {
        throw new Error(`Failed to fetch file: ${response.status}`);
      }
      
      const arrayBuffer = await response.arrayBuffer();
      return new Uint8Array(arrayBuffer);
    } catch (error) {
      console.error('Error converting URI to Uint8Array:', error);
      throw error;
    }
  };
  
  

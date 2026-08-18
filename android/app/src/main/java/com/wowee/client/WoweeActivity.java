package com.wowee.client;

import android.content.pm.PackageManager;
import android.os.Bundle;
import android.system.ErrnoException;
import android.system.Os;
import android.util.Log;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.FileOutputStream;

import org.libsdl.app.SDLActivity;

/**
 * Puts the client's files where the desktop build expects to find them, then
 * hands off to SDL.
 *
 * The client reads its shaders, its interface and its expansion profiles
 * through paths relative to the working directory, which Android does not give
 * a process. So everything the APK carries is unpacked into one root, that root
 * is named in WOWEE_RESOURCE_ROOT, and main() enters it before anything else
 * runs. The layout under it is the layout of a desktop install: assets/ beside
 * Data/.
 *
 * The root is getExternalFilesDir, not filesDir, because the player has to be
 * able to reach it: game data is ~35 GB extracted from their own WoW install on
 * a PC and copied to Data/ under this root, which is visible over USB and to
 * adb push. Nothing here downloads or ships game data.
 */
public class WoweeActivity extends SDLActivity {

    private static final String TAG = "Wowee";

    /** SDLActivity loads these in order; libwowee.so provides SDL_main. */
    @Override
    protected String[] getLibraries() {
        return new String[] { "SDL2", "wowee" };
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        // Everything below has to happen before super.onCreate, which loads the
        // native library and calls into main().
        File root = getExternalFilesDir(null);
        if (root == null) {
            // No external volume mounted. filesDir always exists, and the
            // client will run from it, but the player cannot copy data in.
            Log.e(TAG, "no external files directory; falling back to internal storage");
            root = getFilesDir();
        }

        try {
            if (packagedAssetsAreStale(root)) {
                unpackAssets("assets", new File(root, "assets"));
                unpackAssets("Data", new File(root, "Data"));
                markAssetsUnpacked(root);
            }
        } catch (IOException e) {
            Log.e(TAG, "could not unpack packaged assets", e);
        }

        File data = new File(root, "Data");
        if (!hasGameData(data)) {
            Log.w(TAG, "no game data in " + data.getAbsolutePath()
                    + " - extract it from a WoW install and copy it here.");
        }

        setEnv("WOWEE_RESOURCE_ROOT", root.getAbsolutePath());
        setEnv("WOW_DATA_PATH", data.getAbsolutePath());
        setEnv("WOWEE_CONFIG_ROOT", new File(root, "config").getAbsolutePath());
        new File(root, "config").mkdirs();

        super.onCreate(savedInstanceState);
    }

    private void setEnv(String name, String value) {
        try {
            Os.setenv(name, value, true);
        } catch (ErrnoException e) {
            Log.e(TAG, "could not set " + name, e);
        }
    }

    /**
     * True if anything other than the JSON profiles the APK itself carries is
     * present, which is the cheapest test for "the player copied data in".
     */
    private boolean hasGameData(File data) {
        String[] entries = data.list();
        if (entries == null) return false;
        for (String entry : entries) {
            if (!entry.equals("expansions") && !entry.equals("opcodes")) return true;
        }
        return false;
    }

    /**
     * True when the APK is newer than the last unpack, which is every first run
     * and every install over the top of an older build. Without this a new
     * client would keep running the shaders the old one left on disk.
     */
    private boolean packagedAssetsAreStale(File root) {
        File stamp = new File(root, ".unpacked");
        if (!stamp.isFile()) return true;
        try {
            long installed = getPackageManager()
                    .getPackageInfo(getPackageName(), 0).lastUpdateTime;
            return stamp.lastModified() < installed;
        } catch (PackageManager.NameNotFoundException e) {
            return true;
        }
    }

    private void markAssetsUnpacked(File root) throws IOException {
        File stamp = new File(root, ".unpacked");
        try (OutputStream out = new FileOutputStream(stamp)) {
            out.write('\n');
        }
    }

    /**
     * Copies an APK asset tree out to disk. Files already present are replaced,
     * because this only runs when the APK is newer than what is on disk.
     */
    private void unpackAssets(String assetPath, File dest) throws IOException {
        String[] children = getAssets().list(assetPath);
        if (children == null || children.length == 0) {
            copyAsset(assetPath, dest);
            return;
        }
        if (!dest.isDirectory() && !dest.mkdirs()) {
            throw new IOException("could not create " + dest);
        }
        for (String child : children) {
            unpackAssets(assetPath + "/" + child, new File(dest, child));
        }
    }

    private void copyAsset(String assetPath, File dest) throws IOException {
        File parent = dest.getParentFile();
        if (parent != null) parent.mkdirs();
        byte[] buffer = new byte[64 * 1024];
        try (InputStream in = getAssets().open(assetPath);
             OutputStream out = new FileOutputStream(dest)) {
            int read;
            while ((read = in.read(buffer)) != -1) out.write(buffer, 0, read);
        }
    }
}

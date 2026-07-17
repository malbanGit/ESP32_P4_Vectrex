

bool read_ini_rom_name(char *out_name, int out_size)
{
    FILE *f = fopen(INI_FILE_PATH, "r");
    if (!f) {
        return false;
    }

    if (fgets(out_name, out_size, f) == NULL) {
        fclose(f);
        return false;
    }
    fclose(f);

    // Entferne CR/LF
    char *nl = strpbrk(out_name, "\r\n");
    if (nl) *nl = 0;

    return true;
}


long load_rom_file(const char *rom_name, uint8_t *buffer, long max_size)
{
    char path[256];
    snprintf(path, sizeof(path), "/sdcard/%s", rom_name);

    FILE *f = fopen(path, "rb");
    if (!f) {
        return -1;
    }

    // Datei-Ende bestimmen
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size > max_size) 
	{
        fclose(f);
        return -2;      // Datei größer als Buffer
    }

    long read_bytes = fread(buffer, 1, size, f);
    fclose(f);

    if (read_bytes != size) 
	{
        return -3;      // Fehler beim Lesen
    }

    return size;
}


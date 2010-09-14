byte PLAY_SONG = 1
byte SET_SONG = 2

byte command # Indicates what to do (commands above)
byte song_index # the song index to set or play

uint8 song_length
uint8[] notes
uint8[] note_lengths
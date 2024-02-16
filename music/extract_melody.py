import mido
from mido import MidiFile

def get_bpm_from_midi(midi_file_path):
    midi = MidiFile(midi_file_path)
    for track in midi.tracks:
        for msg in track:
            if msg.type == 'set_tempo':
                # Tempo is in microseconds per beat
                microseconds_per_beat = msg.tempo
                # Convert microseconds per beat to beats per minute (BPM)
                bpm = mido.tempo2bpm(microseconds_per_beat)
                return int(bpm)  # Return BPM as an integer
    return 120  # Return a default value if no set_tempo message is found

def format_notes_as_c_array(notes_and_durations, bpm):
    formatted_notes = ",\n  ".join([f"{note}, {duration}" for note, duration in notes_and_durations])
    return f"const int songTempo = {bpm};\nconst int songMelody[] = {{\n  {formatted_notes},\n}};"

def note_number_to_letter_with_octave(note_number):
    notes = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    note_name = notes[note_number % 12]
    octave = note_number // 12 - 1
    return f"NOTE_{note_name}{octave}".replace('#', 'S')

def midi_ticks_to_note_duration(ticks, ticks_per_beat):
    durations = [(1, 4), (1.5, -4), (0.5, 8), (0.75, -8), (0.25, 16), (0.375, -16)]
    beats = ticks / ticks_per_beat
    closest_duration = min(durations, key=lambda x: abs(x[0] - beats))
    return closest_duration[1]

def extract_notes_and_durations_from_midi(midi_file_path):
    midi = MidiFile(midi_file_path)
    notes_and_durations = []
    elapsed_time = 0

    for track in midi.tracks:
        for msg in track:
            elapsed_time += msg.time

            if msg.type == 'note_on' and msg.velocity > 0:
                if elapsed_time > 0:
                    rest_duration = midi_ticks_to_note_duration(elapsed_time, midi.ticks_per_beat)
                    notes_and_durations.append(('REST', rest_duration))
                    elapsed_time = 0
                note_letter_with_octave = note_number_to_letter_with_octave(msg.note)
            elif (msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0)) and elapsed_time > 0:
                note_duration = midi_ticks_to_note_duration(elapsed_time, midi.ticks_per_beat)
                notes_and_durations.append((note_letter_with_octave, note_duration))
                elapsed_time = 0

    return notes_and_durations

# Example usage
midi_file_path = '/midis/fucik-entrance-of-the-gladiators.mid'
bpm = get_bpm_from_midi(midi_file_path)
notes_and_durations = extract_notes_and_durations_from_midi(midi_file_path)  # Assuming this function is defined as before
formatted_output = format_notes_as_c_array(notes_and_durations, bpm)
print(formatted_output)
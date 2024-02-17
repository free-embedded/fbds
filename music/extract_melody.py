import mido
from mido import MidiFile
import os
import shutil
import argparse

# delete all files in the output directory but not the directory itself
def clear_output_directory(directory):
    for file in os.listdir(directory):
        file_path = os.path.join(directory, file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print(f"Failed to delete {file_path}. Reason: {e}")

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
    return f"const int songMelody[] = {{\n  {formatted_notes},\n}};"

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

# copy header files to out folder from the header folder
def add_header_files(wd,out_dir):
    for header_file in os.listdir(f"{wd}/headers"):    
        shutil.copy2(f'{wd}/headers/{header_file}', f'{out_dir}/{header_file}')   

# process all midi files in the folder
def process_directory(path, out_dir):
    #execute the code for each midi file in the midis folder
    for file in os.listdir(path):
        if file.endswith(".mid"):
            process_midi_file(file.split('.')[0], f"{path}/{file}", out_dir)
    
# process a single midi file
def process_midi_file(midi_file_name, midi_file_path, output_directory):
    # get bpm and notes from midi file
    bpm = get_bpm_from_midi(midi_file_path)
    notes_and_durations = extract_notes_and_durations_from_midi(midi_file_path)

    # format bpm and notes as a .c file
    formatted_output = f"#include \"notes.h\"\n#include \"song.h\"\n\n"             #include the header files
    formatted_output += f"const int songTempo = {bpm};\n\n"                         #add the tempo to the .c file
    formatted_output += format_notes_as_c_array(notes_and_durations, bpm)           #add the melody to the .c file


    # write formatted_output inside .c file (if the file does not exist, create it)
    # use the name of the midi file as the name of the .c file
    with open(f"{output_directory}/{midi_file_name}.c", 'w') as file:
        file.write(formatted_output)


def main():
    # get the script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    wd = (os.getcwd())
    
    
    # Create the parser
    parser = argparse.ArgumentParser(description="Script to extract melody from MIDI files and manage output directory.")
    # Add arguments
    parser.add_argument("-c", "--clear", action="store_true",
                        help="Clear the output directory")
    parser.add_argument("-o", "--output", default=f"{wd}/out", help="Specify custom output folder")
    parser.add_argument("path", nargs="?", default=".",
                        help="Path to the MIDI file or directory to process")

    # Parse the command line arguments
    args = parser.parse_args()
    
    
    output_directory = args.output
    # Ensure the output directory exists
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)
    
    # Handle the clear output directory argument
    if args.clear:
        clear_output_directory(output_directory)
        print("Output directory cleared.")
        return

    # If the specified path is a MIDI file, process it
    if os.path.isfile(args.path):
        if args.path.endswith(".mid"):
            add_header_files(wd,output_directory)
            process_midi_file(args.path.split('.')[0], args.path, output_directory)
        else:
            print("The specified file is not a MIDI file.")
            
    # If the specified path is a directory, process all MIDI files in it
    elif os.path.isdir(args.path):
        # check for default
        if args.path == ".":
            add_header_files(wd,output_directory)
            process_directory(f"{wd}/midis", output_directory)
        else:
            add_header_files(wd,out_dir=output_directory)
            process_directory(args.path, output_directory)
    
    # If the specified path is not valid, print an error message
    else:
        print("The specified path is not valid. Please specify a MIDI file or a directory.")

 
if __name__ == "__main__":
    main()           
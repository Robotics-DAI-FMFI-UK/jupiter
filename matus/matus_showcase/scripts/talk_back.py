#!/usr/bin/env python
from gtts import gTTS
import os
import pygame


class Talker:

    def __init__(self, text):
        self.text = self.text_init(text)
        # Create a text-to-speech object and specify the language
        tts = gTTS(text=self.text, lang='en')
        self.path_to_file = '/home/mustar/jupiter/matus/matus_showcase/audio_files/response.mp3'

        # Save the audio file
        tts.save(self.path_to_file)

    def talk(self):
        pygame.mixer.init()
        pygame.mixer.music.load(self.path_to_file)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue
        pygame.mixer.quit()
        self.delete_audio_file()

    # Delete the audio file
    def delete_audio_file(self):
        try:
            os.remove(self.path_to_file)
            print("Temporary audio file deleted successfully")
        except OSError as e:
            print("Error deleting file:", e)


    def text_init(self, text):
        if text == 'glasses':
            return "This person is wearing glasses."
        if text == 'cap':
            return "Really nice cap, forreal, forreal, no cap."
        if text == 'red_t_shirt':
            return "Nice red T-shirt. You know what color to wear to battle."
        return text


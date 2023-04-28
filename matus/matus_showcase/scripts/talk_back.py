#!/usr/bin/env python
from gtts import gTTS
import os
import pygame


class Talker:

    def __init__(self, text):
        self.text = self.text_init(text)
        # Create a text-to-speech object and specify the language
        tts = gTTS(text=self.text, lang='en')

        # Save the audio file
        tts.save('hello.mp3')

    def talk(self):
        pygame.mixer.init()
        sound_file = "hello.mp3"
        pygame.mixer.music.load(sound_file)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            continue
        pygame.mixer.quit()

    def text_init(self, text):
        if text == 'glasses':
            return "This person is wearing glasses."
        if text == 'cap':
            return "Really nice cap, forreal, forreal, no cap."
        return "Nice red T-shirt. You know what color to wear to battle."


# Hinweise

Hier befinden sich einige Hinweise, wie verschiedene Anforderungen der HAWA umgesetzt werden können:

- Metadaten (Titel, Autor(en), Matrikelnummer(n) usw.) werden in die Datei `metadaten.sty` eingetragen und dann im Dokument und in den PDF-Metadaten verwendet. Die Kommandos dazu können natürlich überall verwendet werden. Sollte ein sehr langer Titel gewählt werden, ist es möglich, dass dieser auf den Titelseiten oder den Erklärungen zu Problemen führt. Diese müssen in den jeweiligen Titelseiten- (verringerte Abstände und/oder Schriftgröße) bzw. Erklärungsdateien (zusätzliche Zeilen) behoben werden.

- Anhand der Metadaten wird automatisch entschieden, welche Versionen der Titelseite und Erklärungen verwendet werden und ob Abstract zur Bachelorarbeit und Freigabeerklärung aktiviert werden.

- Fußnoten sollten durch `\fn{Fußnotentext}`, Online-Zitate durch `\onlinezitat{key}`, andere Zitate durch `\zitat{key}` eingetragen werden. Beispiele dazu, wie Quellen zu speichern sind, sind in `literatur.bib` zu finden.

- Mit `\label{bezeichner:name}` können Bezeichner für Elemente erstellt werden. Auf diese kann über LaTeX-eigene oder in `vorlage/vorlage-commands.tex` definierte Kommandos zugegriffen werden.
    - Die vordefinierten Kommandos sind folgendermaßen aufgebaut:
        - Universelle Kommandos für kurze (`\uniliteref`) bzw. lange (`\unifullref`) Referenzen
        - Typ-spezifische Varianten davon (`\litearef` bzw. `\fullaref`) für Anhänge (a), Abbildungen (b), Codes (c), Formeln (f), Kapitel (=Sektionen, s) unt Tabellen (t)
        - Kurz-Versionen der `lite`-Kommandos, z.B. `\cref` um `\litecref` und dadurch `\uniliteref{Code}` auszulösen

- Pixelgrafiken kann man mittels `\bild[skalierung]{dateiname}{Beschriftung}{label}` einfügen. Vektorgrafiken im SVG-Format analog dazu per `\svg[...]` (besser).

- Abkürzungen werden in `Inhalt/Abkürzungen.tex` eingetragen und im Text bspw. mit `\ac{Kürzel}` verwendet, siehe [Acronym](https://www.namsu.de/Extra/pakete/Acronym.html). Wichtig ist, dass zu Beginn der Umgebung `\begin{acronym}[SSHHH]` in die eckigen Klammern das längste Akronym eingetragen wird. Ansonsten wird die Seite nicht korrekt formatiert.
    - Abkürzungen in der Mehrzahl: `\acp{Kürzel}`
    - erzwungene Kurzform: `\acs{Kürzel}`
    - erzwungene Langform: `\acf{Kürzel}`
    - Abkürzungen in Überschriften ausschließlich ohne `\ac{}`-Kommandos verwenden

- Abkürzungen müssen in der richtigen Reihenfolge eingetragen oder das `sortieren.py`-Script verwendet werden.

- `\vglink{url}{datum}` erzeugt eine Fußnote mit vgl. link (xx.xx.2020) nach der gültigen Formatierung

- Soll Programmcode in der Arbeit angezeigt bzw. eingebunden werden so steht dafür nun die Umgebung `\begin{code}` zur Verfügung. Der genaue Syntax ist folgender:
```latex
\begin{code}[H]
    \inputminted[
        firstline=27,
        lastline=37,
        firstnumber=17,
        numbersep=5pt]{python}{sourcecodes/excel2.py}
    \caption{Auslesen der Daten aus dem Array}
    \label{code:4}
\end{code}
```

- Um Verzeichnisstrukturen darzustellen, wird `\verzeichnis` mit den eigenen Befehlen `\dtfolder` und `\dtfile` genutzt:
  - Dieses Kommando basiert auf Dirtree, in dessen [Doku](http://tug.ctan.org/macros/generic/dirtree/dirtree.pdf) weitere nützilche Kommandos erläutert werden.
```latex
\verzeichnis{%
    .1 \dtfolder Ordner 1. %Ebene 1
    .2 \dtfolder Unterordner 1. %Ebene 2
    .3 \dtfile Datei 1. %Ebene 3
    .2 \dtfolder Unterordner 2. %Ebene 2
    .3 \dtfolder Unter-Unterordner 1. %Ebene 3
    .4 \dtfile Datei 2. %Ebene 4
    .3 \dtfolder Unter-Unterordner 2. %Ebene 3
}{Beschriftung}{label}
```

- Für mehr Informationen dazu bitte die Minted-Dokumentation konsultieren. Es ist möglich mittels ` \mintinline{python}{print("Toller Code")}` Code in einer Zeile ähnlich dem Mathemathikmodus zu verwenden.

- Die Formelumgebung wird wie folgt aufgerufen:
```latex
\begin{formel}[H]
\captionabove{Die erste aller Formeln}
$[\lnot P\land\lnot Q] \lor R$ \vspace{3mm}\\
$P=\text{was ganz tolles}$ \\
$Q=\text{was noch viel tolleres}$
\label{formel:test}
\end{formel}
```

- Soll die Caption darunter stehen muss `\captionabove{}` durch `\caption{}` ersetzt werden und über `\label{}` geschrieben werden.

- Die einzelnen Zeilen der Formel werden im Mathemathikmodus geschrieben. Die Erklärung der Formelzeichen auch. Siehe dazu auch das HAWA-Dokument.

- Normale Anführungszeichen ("") entsprechen nicht der Deutschen Rechtschreibung. Hierzu das Kommando `\striche{Text}` verwenden.

- Reverse SyncTeX: Um aus der PDF-Ansicht an die entsprechende Stelle im Code zu gelangen, diese mit gehaltener STRG-Taste anklicken.

- Forward SyncTeX: Um aus dem Code an die entsprechende Stelle der PDF zu gelangen, STRG+ALT+J drücken. Hierzu muss allerdings die Tastenkombination bearbeitet werden.
  -  hintereinander STRG+K, STRG+S drücken
  -  im erscheinenden Fenster "Tastenkombinationen" nach "Synctex from cursor" suchen
  -  Rechtsklick auf die Zeile mit Tastenzuordnung "STRG+ALT+J" --> "when-Ausdruck ändern"
  -  `editorTextFocus && !config.latex-workshop.bind.altKeymap.enabled && editorLangId == 'latex'` durch `editorTextFocus && editorLangId == 'latex'` ersetzen (bzw. `!config.latex-workshop.bind.altKeymap.enabled &&` entfernen)
  -  mit Enter bestätigen
  -  alternativ kann folgender Codeblock in die "keybindings.json" des Benutzers einfügen: (STRG+UMSCH+P --> Einstellungen: Tastenkombinationen öffnen (JSON))
```json
[
    {
        "key": "ctrl+alt+j",
        "command": "latex-workshop.synctex",
        "when": "editorTextFocus && editorLangId == 'latex'"
    },
    {
        "key": "ctrl+alt+j",
        "command": "-latex-workshop.synctex",
        "when": "editorTextFocus && !config.latex-workshop.bind.altKeymap.enabled && editorLangId == 'latex'"
    }
]
```

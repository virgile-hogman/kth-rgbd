%%
%% This is file `kthesis.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% kthesis.dtx  (with options: `classfile,drlic,avhandling')
%% 
%% IMPORTANT NOTICE:
%% 
%% For the copyright see the source file.
%% 
%% Any modified versions of this file must be renamed
%% with new filenames distinct from kthesis.cls.
%% 
%% For distribution of the original source see the terms
%% for copying and modification in the file kthesis.dtx.
%% 
%% This generated file may be distributed as long as the
%% original source files, as listed above, are part of the
%% same distribution. (The sources need not necessarily be
%% in the same archive or directory.)
\NeedsTeXFormat{LaTeX2e}[2001/01/01]
\ProvidesClass{kthesis}
              [2004/12/14 v2.1 Document Class for KTH Theses]
\newif\if@kthesis@blurb \@kthesis@blurbtrue
\DeclareOption{noblurb}{\@kthesis@blurbfalse}
\newif\if@kthesis@electronic \@kthesis@electronicfalse
\DeclareOption{electronic}{\@kthesis@electronictrue}
\def\@kthesis@setpapersize@afour{%
  \setstocksize{297mm}{210mm}
  \settrimmedsize{297mm}{210mm}{*}
  \settypeblocksize{49pc}{33pc}{*}}
\def\@kthesis@setpapersize@gfive{%
  \if@kthesis@electronic
    \setstocksize{239mm}{169mm}
    \settrimmedsize{239mm}{169mm}{*}
    \ifpdf
    \else
      \special{papersize=169mm,239mm}
    \fi
  \else
    \setstocksize{297mm}{210mm}
    \settrimmedsize{239mm}{169mm}{*}
    \setlength{\trimtop}{\stockheight}
    \addtolength{\trimtop}{-\paperheight}
    \setlength{\trimtop}{0.5\trimtop}
    \setlength{\trimedge}{\stockwidth}
    \addtolength{\trimedge}{-\paperwidth}
    \ifpdf
      \AtBeginDocument{%
        \setlength{\pdfpagewidth}{\stockwidth}
        \setlength{\pdfpageheight}{\stockheight}}
    \fi
  \fi
  \settypeblocksize{43pc}{30pc}{*}}
\DeclareOption{g5paper}{\def\kthesis@trimmedsize{gfive}}
\DeclareOption{a3paper}{\OptionNotUsed}
\DeclareOption{a4paper}{\def\kthesis@trimmedsize{afour}}
\DeclareOption{a6paper}{\OptionNotUsed}
\DeclareOption{a5paper}{\OptionNotUsed}
\DeclareOption{b3paper}{\OptionNotUsed}
\DeclareOption{b4paper}{\OptionNotUsed}
\DeclareOption{b5paper}{\OptionNotUsed}
\DeclareOption{b6paper}{\OptionNotUsed}
\DeclareOption{letterpaper}{\OptionNotUsed}
\DeclareOption{legalpaper}{\OptionNotUsed}
\DeclareOption{executivepaper}{\OptionNotUsed}
\DeclareOption{ebook}{\OptionNotUsed}
\DeclareOption{landscape}{\OptionNotUsed}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{memoir}}
\ExecuteOptions{g5paper}
\ProcessOptions\relax
\LoadClass[a4paper]{memoir}
\AtBeginDocument{\RequirePackage{graphics}}
\@nameuse{@kthesis@setpapersize@\kthesis@trimmedsize}
\setlrmargins{*}{*}{*}
\setulmargins{*}{*}{*}
\checkandfixthelayout
\def\subtitle{\gdef\thesubtitle}
\def\thesistype{\gdef\thethesistype}
\def\examen{\gdef\theexamen}
\def\disputationsdatum{\gdef\thedisputationsdatum}
\def\disputationslokal{\gdef\thedisputationslokal}
\def\imprint{\gdef\theimprint}
\def\isbn{\gdef\theisbn}
\def\isrn{\gdef\theisrn}
\def\issn{\gdef\theissn}
\def\publisher{\gdef\thepublisher}
\def\address{\gdef\theaddress}
\def\trita{\gdef\thetrita}
\def\kthlogo{\gdef\thekthlogo}
\let\thesubtitle\relax
\def\thethesistype{\ClassError{kthesis}%
  {\protect\thesistype\space not given}%
  {Please Read The Fine Manual.}}
\def\theexamen{\ClassError{kthesis}%
  {\protect\examen\space not given}%
  {Please Read The Fine Manual.}}
\def\thedisputationsdatum{\ClassError{kthesis}
  {\protect\thedisputationsdatum\space not given}%
  {Please Read The Fine Manual.}}
\def\thedisputationslokal{\ClassError{kthesis}
  {\protect\thedisputationslokal\space not given}%
  {Please Read The Fine Manual.}}
\def\theimprint{\ClassError{kthesis}%
  {\protect\theimprint\space not given}%
  {Please Read The Fine Manual.}}
\def\thepublisher{\ClassError{kthesis}%
  {\protect\thepublisher\space not given}%
  {Please Read The Fine Manual.}}
\def\theaddress{KTH\\SE-100 44 Stockholm\\SWEDEN}
\def\thekthlogo{kth_svv}
\makepagestyle{kthabstract}
\makeevenhead{kthabstract}{}{}{}
\makeoddhead{kthabstract}{}{}{}
\makeevenfoot{kthabstract}{}{\normalfont\scriptsize
  \def\@maybebullet{\if@bullet\space\textbullet\space\fi}
  \newif\if@bullet \@bulletfalse
  \@ifundefined{thetrita}{}{\@maybebullet\thetrita\@bullettrue}%
  \@ifundefined{theissn}{}{\@maybebullet\theissn\@bullettrue}%
  \@ifundefined{theisrn}{}{\@maybebullet\theisrn\@bullettrue}%
  \@ifundefined{theisbn}{}{\@maybebullet\theisbn\@bullettrue}%
}{}
\makeoddfoot{kthabstract}{}{\normalfont\scriptsize
  \def\@maybebullet{\if@bullet\space\textbullet\space\fi}
  \newif\if@bullet \@bulletfalse
  \@ifundefined{thetrita}{}{\@maybebullet\thetrita\@bullettrue}%
  \@ifundefined{theissn}{}{\@maybebullet\theissn\@bullettrue}%
  \@ifundefined{theisrn}{}{\@maybebullet\theisrn\@bullettrue}%
  \@ifundefined{theisbn}{}{\@maybebullet\theisbn\@bullettrue}%
}{}
\newif\if@kthesis@babel \@kthesis@babelfalse
\AtBeginDocument{%
  \@ifpackagewith{babel}{swedish}%
     {\@kthesis@babeltrue}{\@kthesis@babelfalse}}
\def\maketitle{%
  \begingroup
  \sffamily
  \pagestyle{empty}
  \renewcommand*{\thepage}{title-\arabic{page}}
  \noindent
  \parbox[b][0.6\textheight]{\textwidth}{\centering\noindent
    \scalebox{0.192}{\includegraphics{\thekthlogo}}\par
    \vskip1cm
    \LARGE\bfseries\thetitle\par
    \vfill
    \normalsize\mdseries\thesubtitle\par
    \vfill
    \large\MakeUppercase{\theauthor}\par
  }\par
  \vfill\noindent
  \parbox[b]{\textwidth}{\centering\noindent\large
    \thethesistype\par
    \theimprint\par
  }\par
  \newpage
  \hbox{}\vfill
  \rmfamily
  \noindent\begin{tabular}[b]{@{}l@{}}
    \@ifundefined{thetrita}{}{\thetrita\\}%
    \@ifundefined{theissn}{}{\theissn\\}%
    \@ifundefined{theisrn}{}{\theisrn\\}%
    \@ifundefined{theisbn}{}{\theisbn\\}%
  \end{tabular}
  \hfill
  \noindent\begin{tabular}[b]{@{}r@{}}
    \theaddress
  \end{tabular}
  \par\bigskip
  \if@kthesis@blurb
    \if@kthesis@babel
      \begin{otherlanguage}{swedish}
        \noindent Akademisk avhandling som med tillst\aa nd
        av Kungl Tekniska h\"{o}gskolan framl\"{a}gges till
        offentlig granskning f\"{o}r avl\"aggande av
        \theexamen\space \thedisputationsdatum\space i
        \thedisputationslokal.\par
      \end{otherlanguage}
    \else
      \noindent Aka\-de\-misk av\-hand\-ling som med
      till\-st\aa nd av Kungl Tek\-niska h\"{o}g\-skolan
      fram\-l\"{a}g\-ges till of\-fent\-lig gransk\-ning
      f\"{o}r av\-l\"ag\-gande av \theexamen\space
      \thedisputationsdatum\space i \thedisputationslokal.\par
    \fi
    \bigskip
  \fi
  \noindent
  \copyright \space \theauthor, \thedate
  \par\bigskip\noindent
  Tryck: \thepublisher\par
  \cleardoublepage
  \endgroup
}
\endinput
%%
%% End of file `kthesis.cls'.

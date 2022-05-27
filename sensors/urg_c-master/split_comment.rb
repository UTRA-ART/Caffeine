#!/usr/bin/env ruby
# -*- coding: cp932 -*-

# Doxygen �R�����g�ɏ]���đΏۃt�@�C�����R�s�[����Ƃ��̃R�����g�o�͂𒲐�����

#$KCODE = "SJIS"


if ARGV.empty?
  print "usage: \n" +
    "    " + __FILE__ + " [options] <files> <directory> \n" +
    "\n" +
    "[options]\n" +
    " -e      output english comment.\n" +
    " -j      output Japanese comment.\n" +
    "\n"
  exit(1)
end


# �Ō�̈������f�B���N�g���łȂ���΁A�G���[���b�Z�[�W��Ԃ�
output_directory = ARGV.pop
output_mode = ARGV.shift
if not FileTest::directory?(output_directory)
  print "No such directory: " + output_directory + "\n"
  exit(1)
end
target_files = ARGV


def split_single_line(line, mode, output_mode)

  # ���݂̃��[�h�ɏ]�����R�����g�݂̂��o�͂���
  if output_mode == "-e" then
    if line =~ /\\\~japanese .+ ([\*\\])/
      line = $` + $1 + $'
    end
    if line =~ /\\\~english /
      line = $` + $'
    end
  end

  if output_mode == "-j"
    if line =~ /\\\~english .+ ([\*\\])/
      line = $` + $1 + $'
    end
    if line =~ /\\\~japanese /
      line = $` + $'
    end
  end

  return line
end


def remove_matched_word(line)

  if line.strip == ""
    line = ""
  end
  line
end


# Doxygen �R�����g�ɏ]���đΏۃt�@�C�����R�s�[����Ƃ��̃R�����g�o�͂𒲐�����
def split_comment(file_name, output_mode)
  lines = ""
  mode = "both"
  is_comment = false

  p file_name
  File.open(file_name) { |fd|
    fd.each { |line|
      line.force_encoding("cp932")

      if is_comment
        # �R�����g��
        case line
        when /\\\~japanese/
          mode = "japanese"
          line = $` + $'
          if line.strip == ""
            line = ""
          end
        when /\\\~english/
          mode = "japanese"
          line = $` + $'
          if line.strip == ""
            line = ""
          end
        when /\\\~/
          mode = "both"
          line = remove_matched_word($` + $')
        end
        if line =~ /\*\//
          is_comment = false
          lines += line
        else
          # ���[�h�ɏ]���ăR�����g���o�͂���
          if mode == "japanese" and output_mode == "-j"
            lines += line
          elsif mode == "english" and output_mode == "-e"
            lines += line
          elsif mode == "both"
            lines += line
          end
        end
      else
        # �\�[�X�R�[�h��
        if line =~ /\/\*/
          if line =~ /\*\//
            is_comment = false
            line = split_single_line(line, mode, output_mode)
          else
            is_comment = true
            mode = "both"
          end
        else
          # // �R�����g�̏ꍇ�̏���
          case line
          when /~japanese/
            line = split_single_line(line, mode, output_mode)
          when /~english/
            line = split_single_line(line, mode, output_mode)
          end
        end
        lines += line
      end
    }
  }

  lines
end


# �Ώۂ̃t�@�C�����������A�w�肳�ꂽ�f�B���N�g���ɃR�s�[����
target_files.each { |file_name|
  converted_lines = split_comment(file_name, output_mode)
  converted_file_name = output_directory + "/" + File.basename(file_name)

  File.open(converted_file_name, "w") { |fd|
    fd.print converted_lines
  }
}

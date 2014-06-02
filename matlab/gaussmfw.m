function z = gaussmfw(x, p)
  z = 1/(p(1)*sqrt(2*pi)) * gaussmf(x, p);
end

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)
